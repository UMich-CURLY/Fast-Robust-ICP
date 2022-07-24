
cd build
make -j
cd ..
export OMP_NUM_THREADS=36
methods=(ICP AA_ICP FICP RICP PPL RPPL SparseICP SICPPPL)
#        0    1      2    3    4   5    6         7
for method in   3 2 6
do
	for seq in 00 01 02 03 04 05 06 07 08 09 10
	do
		echo "kitti/$seq/${methods[${method}]}"
		mkdir -p kitti/$seq/${methods[${method}]}
		#gdb -ex run --args \
		./build/kitti_test  /home/rzh/media/sdg1/kitti/kitti/sequences/$seq kitti/$seq/${methods[${method}]}/traj.txt \
		       kitti/$seq/${methods[${method}]}/time.txt  0 100000 $method	
		sleep 2
	done

done
