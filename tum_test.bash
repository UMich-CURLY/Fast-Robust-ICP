
cd build
make -j
cd ..
export OMP_NUM_THREADS=36
methods=(ICP AA_ICP FICP RICP PPL RPPL SparseICP SICPPPL)
#        0    1      2    3    4   5    6         7
for method in  3 2 6
do
	for seq in  freiburg1_360 freiburg1_desk freiburg1_desk2 freiburg1_floor freiburg1_plant freiburg1_room freiburg1_rpy freiburg1_teddy freiburg1_xyz freiburg3_nostructure_notexture_far freiburg3_nostructure_notexture_near freiburg3_nostructure_texture_far freiburg3_nostructure_texture_near/  freiburg3_structure_notexture_far/  freiburg3_structure_notexture_near/ freiburg3_structure_texture_far/ freiburg3_structure_texture_near/
	do
		echo "tum/$seq/${methods[${method}]}"
		mkdir -p tum/$seq/${methods[${method}]}
		#gdb -ex run --args \
		./build/tum_test  /home/rzh/media/sdg1/tum/$seq/ tum/$seq/${methods[${method}]}/traj.txt \
		       tum/$seq/${methods[${method}]}/time.txt  0 100000 $method	
		sleep 1
	done

done
