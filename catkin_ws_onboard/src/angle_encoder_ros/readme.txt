sudo apt install odroid-wiringpi
gpio readall

cd Documents/encoder/
python3 wp_enc.py 

git clone https://github.com/hardkernel/wiringPi.git
cd wiringPi/
./build
cd ..
gcc -o wp_enc wp_enc.c -lwiringPi -lwiringPiDev -lm -lrt -lcrypt
./wp_enc 

pip install odroid-wiringpi