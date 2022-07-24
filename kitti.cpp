#include <iostream>
#include <string>
#include <fstream>
#include <memory>
#include <Eigen/Core>
#include "ICP.h"
#include "io_pc.h"
#include "FRICP.h"
#include "dataset_handler/KittiHandler.hpp"
//#include "cvo/Calibration.hpp"
#include <utils/CvoPointCloud.hpp>
#include "utils/ImageStereo.hpp"
#include "utils/Calibration.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <vector>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vertices;
typedef Eigen::Matrix<Scalar, 3, 1> VectorN;


void CvoPointCloud_to_Vertices(const cvo::CvoPointCloud & pc_in,
                               std::shared_ptr<Vertices> out) {
  out->conservativeResize(3, pc_in.size());
  for (int i = 0; i < pc_in.size(); i++) {
    Eigen::Vector3d p = pc_in.at(i).cast<double>();
    out->row(i) = p;
  }
}

int main(int argc, char const ** argv)
{

  cvo::KittiHandler kitti(argv[1], 1);
  int total_iters = kitti.get_total_number();
  std::string calib_file;
  calib_file = std::string(argv[1] ) +"/cvo_calib.txt"; 
  cvo::Calibration calib(calib_file);
  std::ofstream accum_output(argv[3]);
  std::ofstream time_output(argv[4]);
  int start_frame = std::stoi(argv[5]);
  kitti.set_start_index(start_frame);
  int max_num = std::stoi(argv[6]);

  int method_ind = std::stoi(argv[7]);
  enum Method{ICP, AA_ICP, FICP, RICP, PPL, RPPL, SparseICP, SICPPPL};
  Method method = static_cast<Method>(method_ind);

  Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();  // from source frame to the target frame
  //init_guess(2,3)=2.22;
  Eigen::Matrix4d accum_mat = Eigen::Matrix4d::Identity();

  cv::Mat source_left, source_right;
  //std::vector<float> semantics_source;
  //kitti.read_next_stereo(source_left, source_right, 19, semantics_source);
  kitti.read_next_stereo(source_left, source_right);
  std::cout<<"read source raw...\n";
  std::shared_ptr<cvo::ImageStereo> source_raw(new cvo::ImageStereo(source_left, source_right));
  std::cout<<"build source CvoPointCloud...\n";
  std::shared_ptr<cvo::CvoPointCloud> source_cvo(new cvo::CvoPointCloud(*source_raw, calib
                                                                        , cvo::CvoPointCloud::FULL
                                                                        ));
  std::shared_ptr<Vertices> vertices_source(new Vertices);
  CvoPointCloud_to_Vertices(*source_cvo, vertices_source);

  double total_time = 0;
  int i = start_frame;
  for (; i<std::min(total_iters, start_frame+max_num)-1 ; i++) {
    
     // calculate initial guess
    std::cout<<"\n\n\n\n============================================="<<std::endl;
    std::cout<<"Aligning "<<i<<" and "<<i+1<<" with GPU "<<std::endl;

    kitti.next_frame_index();
    cv::Mat left, right;
    //vector<float> semantics_target;
    //if (kitti.read_next_stereo(left, right, 19, semantics_target) != 0) {
    if (kitti.read_next_stereo(left, right) != 0) {
      std::cout<<"finish all files\n";
      break;
    }

    std::shared_ptr<cvo::ImageStereo> target_raw(new cvo::ImageStereo(left, right));
    std::shared_ptr<cvo::CvoPointCloud> target_cvo(new cvo::CvoPointCloud(*target_raw, calib
                                                                          ,cvo::CvoPointCloud::FULL
                                                                          ));
    std::shared_ptr<Vertices> vertices_target(new Vertices);
    CvoPointCloud_to_Vertices(*target_cvo, vertices_target);
   
    bool use_init = true;
    MatrixXX res_trans;

    int dim = 3;


    //--- Model that will be rigidly transformed
    Vertices normal_source, src_vert_colors;
    //read_file(vertices_source, normal_source, src_vert_colors, file_source);
    std::cout << "source: " << vertices_source->rows() << "x" << vertices_source->cols() << std::endl;

    //--- Model that source will be aligned to
    Vertices normal_target, tar_vert_colors;
    //read_file(vertices_target, normal_target, tar_vert_colors, file_target);
    std::cout << "target: " << vertices_target->rows() << "x" << vertices_target->cols() << std::endl;

    // scaling
    Eigen::Vector3d source_scale, target_scale;
    source_scale = vertices_source->rowwise().maxCoeff() - vertices_source->rowwise().minCoeff();
    target_scale = vertices_target->rowwise().maxCoeff() - vertices_target->rowwise().minCoeff();
    double scale = std::max(source_scale.norm(), target_scale.norm());
    std::cout << "scale = " << scale << std::endl;
    *vertices_source /= scale;
    *vertices_target /= scale;

    /// De-mean
    VectorN source_mean, target_mean;
    source_mean = vertices_source->rowwise().sum() / double(vertices_source->cols());
    target_mean = vertices_target->rowwise().sum() / double(vertices_target->cols());
    vertices_source->colwise() -= source_mean;
    vertices_target->colwise() -= target_mean;

    double time;
    // set ICP parameters
    ICP::Parameters pars;

    // set Sparse-ICP parameters
    SICP::Parameters spars;
    spars.p = 0.4;
    spars.print_icpn = false;

    /// Initial transformation
    if(use_init) {
      MatrixXX init_trans = init_guess;
      //read_transMat(init_trans, file_init);
      init_trans.block(0, dim, dim, 1) /= scale;
      init_trans.block(0,3,3,1) += init_trans.block(0,0,3,3)*source_mean - target_mean;
      pars.use_init = true;
      pars.init_trans = init_trans;
      spars.init_trans = init_trans;
    }

    ///--- Execute registration
    std::cout << "begin registration..." << std::endl;
    FRICP<3> fricp;
    double begin_reg = omp_get_wtime();
    double converge_rmse = 0;
    switch(method)
    {
    case ICP:
      {
        pars.f = ICP::NONE;
        pars.use_AA = false;
        fricp.point_to_point(*vertices_source, *vertices_target, source_mean, target_mean, pars);
        res_trans = pars.res_trans;
        break;
      }
    case AA_ICP:
      {
        AAICP::point_to_point_aaicp(*vertices_source, *vertices_target, source_mean, target_mean, pars);
        res_trans = pars.res_trans;
        break;
      }
    case FICP:
      {
        pars.f = ICP::NONE;
        pars.use_AA = true;
        fricp.point_to_point(*vertices_source, *vertices_target, source_mean, target_mean, pars);
        res_trans = pars.res_trans;
        break;
      }
    case RICP:
      {
        pars.f = ICP::WELSCH;
        pars.use_AA = true;
        fricp.point_to_point(*vertices_source, *vertices_target, source_mean, target_mean, pars);
        res_trans = pars.res_trans;
        break;
      }
    case PPL:
      {
        pars.f = ICP::NONE;
        pars.use_AA = false;
        if(normal_target.size()==0)
        {
          std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
          exit(0);
        }
        fricp.point_to_plane(*vertices_source, *vertices_target, normal_source, normal_target, source_mean, target_mean, pars);
        res_trans = pars.res_trans;
        break;
      }
    case RPPL:
      {
        pars.nu_end_k = 1.0/6;
        pars.f = ICP::WELSCH;
        pars.use_AA = true;
        if(normal_target.size()==0)
        {
          std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
          exit(0);
        }
        fricp.point_to_plane_GN(*vertices_source, *vertices_target, normal_source, normal_target, source_mean, target_mean, pars);
        res_trans = pars.res_trans;
        break;
      }
    case SparseICP:
      {
        SICP::point_to_point(*vertices_source, *vertices_target, source_mean, target_mean, spars);
        res_trans = spars.res_trans;
        break;
      }
    case SICPPPL:
      {
        if(normal_target.size()==0)
        {
          std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
          exit(0);
        }
        SICP::point_to_plane(*vertices_source, *vertices_target, normal_target, source_mean, target_mean, spars);
        res_trans = spars.res_trans;
        break;
      }
    }
    double end_reg = omp_get_wtime();
    total_time += end_reg - begin_reg;
    std::cout << "Registration done in " <<end_reg - begin_reg <<" seconds "<< std::endl;    
    *vertices_source = scale * *vertices_source;

    Eigen::Matrix4d res_T = Eigen::Matrix4d::Identity();
    res_trans.block(0,3,3,1) *= scale;    
    res_T.block(0,0,3,3) = res_trans.block(0,0,3,3);
    res_T.block(0,3,3,1) = res_trans.block(0,3,3,1);
    std::cout<<"Transform is "<<res_T <<"\n\n";

    init_guess = res_T;

    accum_mat = (accum_mat * res_T).eval();
    accum_output << accum_mat(0,0)<<" "<<accum_mat(0,1)<<" "<<accum_mat(0,2)<<" "<<accum_mat(0,3)<<" "
                <<accum_mat(1,0)<<" " <<accum_mat(1,1)<<" "<<accum_mat(1,2)<<" "<<accum_mat(1,3)<<" "
                <<accum_mat(2,0)<<" " <<accum_mat(2,1)<<" "<<accum_mat(2,2)<<" "<<accum_mat(2,3);
    accum_output<<"\n";
    accum_output<<std::flush;
    
    vertices_source = vertices_target;
    
    //out_trans << res_trans << std::endl;
    //out_trans.close();
    
  }

  accum_output.close();

  time_output << total_time / static_cast<double>(i - start_frame);
  //out_path = out_path + "m" + std::to_string(method);
  time_output.close();
  return 0;
}
