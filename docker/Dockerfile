FROM gustavovelascoh/orb_slam2:build
# Or use your own build image
# FROM orb_slam2:build
LABEL maintainer="gustavovelascoh@gmail.com"

WORKDIR /opt
RUN cd /opt \
&& git clone https://github.com/gustavovelascoh/ORB_SLAM2.git ORB_SLAM2 \
&& cd ORB_SLAM2 && chmod +x build.sh && sh build.sh
