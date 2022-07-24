
cd build
make -j
cd ..
export OMP_NUM_THREADS=36
methods=(ICP AA_ICP FICP RICP PPL RPPL SparseICP SICPPPL)
#        0    1      2    3    4   5    6         7
for method in  3 2 6
do
	for seq in hospital abandonedfactory_night abandonedfactory gascola soulcity abandonedfactory  seasidetown  seasonsforest seasonsforest_winter hospital endofworld neighborhood oldtown neighborhood  
	do
		echo "tartan/$seq/${methods[${method}]}"
		mkdir -p tartan/$seq/${methods[${method}]}
		#gdb -ex run --args \
		./build/tartan_test  /home/rzh/media/sdg1/tartanair/$seq/Easy/P001/ tartan/$seq/${methods[${method}]}/traj.txt \
		       tartan/$seq/${methods[${method}]}/time.txt  0 100000 $method	
		sleep 2
	done

done
