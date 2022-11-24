# 로라 라즈베리파이

raspberrypi 4 + Ubuntu 22.04

[https://lloydrochester.com/post/hardware/ebyte-e32-lora-configuration-wiring/](https://lloydrochester.com/post/hardware/ebyte-e32-lora-configuration-wiring/)

`sudo apt update`

`sudo apt install raspi-config`

다음에 인터페이스에서 시리얼 포트 → login shell은 NO, 시리얼 활성화는 YES

`sudo reboot`

[https://blog.projectdh.link/108](https://blog.projectdh.link/108)

그 다음 /boot/firmware/config.txt, /boot/firmware/cmdline.txt

블루투스 중지 : 

sudo systemctl stop hciuart

sudo reboot

확인방법 : ls -l /dev

시리얼 통신속도 조정 : sudo stty -F /dev/ttyAMA0 9600