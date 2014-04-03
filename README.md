v3d2
====
dkms version of v3d2 driver, pulled out of my fork of linux

to compile:

`sudo apt-get install linux-headers-3.6-trunk-rpi linux-image-3.6-trunk-rpi dkms`

`cd /usr/src/`

`git clone https://github.com/cleverca22/v3d2.git/ v3d2-0.1`

`dkms add v3d2/0.1`

`dkms build v3d2/0.1 -k 3.6-trunk-rpi`

`dkms install v3d2/0.1 -k 3.6-trunk-rpi`
