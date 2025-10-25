# if u want help
darwin op
https://emanual.robotis.com/docs/en/platform/op/getting_started/

make -f Makefile.darwin-op


segmentaion fault 
make clean -f Makefile.darwin-op



1.sudo su #this is super admin to uesr


2.password : 111111


3.cd .. # go /darwin not /home/darwin


4.killall demo


5.cd Linow/project/dxl_montior/


6. ./dxl_montior


7.scan


8.reset => exit 그리고 로봇 세번째 버튼 누르기


9.cd ../webot/controllers/my_walk



10. 본컴에서 fillzila on



11. back to robot and do ifconfig # cheak inet address ex : 112.162.229.197  



12. paste fillzila at address and id:darwinpw:111111 port:


 
13. ./my_walk and 112.162.229.197:8080



14. if you want build make clean -f Makefile.darwin-op ,  make -f Makefile.darwin-op

 리눅스에서 IP 주소를 다시 받는 방법을 알려드리겠습니다.
DHCP로 IP 재할당 받기
방법 1: dhclient 사용
bashsudo dhclient -r  # 현재 IP 해제
sudo dhclient     # 새 IP 받기
방법 2: 한 번에 실행
bashsudo dhclient -r eth0 && sudo dhclient eth0
(eth0는 네트워크 인터페이스 이름 - 환경에 따라 eth0, ens33, enp0s3 등으로 다를 수 있음)
방법 3: systemd 기반 시스템
bashsudo systemctl restart NetworkManager
방법 4: ifdown/ifup 사용
bashsudo ifdown eth0 && sudo ifup eth0
방법 5: ip 명령어 사용
bashsudo ip link set eth0 down
sudo ip link set eth0 up
sudo dhclient eth0
네트워크 인터페이스 이름 확인
먼저 인터페이스 이름을 확인하려면:
baship a
# 또는
ifconfig
보통 eth0, ens33, enp0s3, wlan0(무선) 등으로 표시됩니다.
어떤 방법이 가장 편하신가요? 사용 중인 리눅스 배포판이 궁금하면 더 구체적으로 알려드릴 수 있습니다!
