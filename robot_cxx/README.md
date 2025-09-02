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
15. 
