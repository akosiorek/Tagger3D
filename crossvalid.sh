#!/bin/bash

echo "crossvalid starting" > crs.log

echo "0.0001, 0.0001" >> crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.0001 --gamma 0.0001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.0001 --gamma 0.001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.0001 --gamma 0.01; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.0001 --gamma 0.1; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.0001 --gamma 1.0; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.0001 --gamma 10; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.0001 --gamma 100; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.0001 --gamma 1000; ./Tagger3D ../config/cv_confing.ini pred>>crs.log

echo "0.001, 0.0001" >> crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.001 --gamma 0.0001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.001 --gamma 0.001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.001 --gamma 0.01; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.001 --gamma 0.1; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.001 --gamma 1.0; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.001 --gamma 10; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.001 --gamma 100; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.001 --gamma 1000; ./Tagger3D ../config/cv_confing.ini pred>>crs.log

echo "0.01, 0.0001" >> crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.01 --gamma 0.0001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.01 --gamma 0.001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.01 --gamma 0.01; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.01 --gamma 0.1; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.01 --gamma 1.0; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.01 --gamma 10; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.01 --gamma 100; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.01 --gamma 1000; ./Tagger3D ../config/cv_confing.ini pred>>crs.log

echo "0.1, 0.0001" >> crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 0.0001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 0.001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 0.01; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 0.1; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 1.0; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 10; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 100; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 1000; ./Tagger3D ../config/cv_confing.ini pred>>crs.log

echo "1, 0.0001" >> crs.log
./Tagger3D ../config/cv_confing.ini train --C 1.0 --gamma 0.0001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 1.0 --gamma 0.001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 1.0 --gamma 0.01; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 1.0 --gamma 0.1; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 1.0 --gamma 1.0; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 1.0 --gamma 10; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 1.0 --gamma 100; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 1.0 --gamma 1000; ./Tagger3D ../config/cv_confing.ini pred>>crs.log

echo "10, 0.0001" >> crs.log
./Tagger3D ../config/cv_confing.ini train --C 10 --gamma 0.0001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10 --gamma 0.001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10 --gamma 0.01; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10 --gamma 0.1; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10 --gamma 1.0; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10 --gamma 10; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10 --gamma 100; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10 --gamma 1000; ./Tagger3D ../config/cv_confing.ini pred>>crs.log

echo "100, 0.0001" >> crs.log
./Tagger3D ../config/cv_confing.ini train --C 100 --gamma 0.0001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 100 --gamma 0.001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 100 --gamma 0.01; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 100 --gamma 0.1; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 100 --gamma 1.0; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 100 --gamma 10; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 100 --gamma 100; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 100 --gamma 1000; ./Tagger3D ../config/cv_confing.ini pred>>crs.log

echo "1000, 0.0001" >> crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 0.0001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 0.001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 0.01; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 0.1; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 1.0; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 10; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 100; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 0.1 --gamma 1000; ./Tagger3D ../config/cv_confing.ini pred>>crs.log

echo "10000, 0.0001" >> crs.log
./Tagger3D ../config/cv_confing.ini train --C 10000 --gamma 0.0001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10000 --gamma 0.001; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10000 --gamma 0.01; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10000 --gamma 0.1; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10000 --gamma 1.0; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10000 --gamma 10; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10000 --gamma 100; ./Tagger3D ../config/cv_confing.ini pred>>crs.log
./Tagger3D ../config/cv_confing.ini train --C 10000 --gamma 1000; ./Tagger3D ../config/cv_confing.ini pred>>crs.log

