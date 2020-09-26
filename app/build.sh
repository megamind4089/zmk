# west build -d build/left  -b nrfmicro_13 -- -DSHIELD=lily58_left
# west build --pristine -t menuconfig -d build/right -b nrfmicro_13 -- -DSHIELD=lily58_right -DKEYMAP=default
# west build -d build/right -b nrfmicro_13 -- -DSHIELD=lily58_right
# west build -d build -b nrfmicro_13 -- -DSHIELD=lily58_left
# cp ./build/zephyr/zmk.uf2 /run/media/maari/NRF52BOOT

# west build -d build -b blackpill_f401cc -- -DSHIELD=2key
west build -d build -b bluepill -- -DSHIELD=2key
