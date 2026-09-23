# xgo_robot_kit_playing_fetch
Project to use XGO Robot Kit with computer vision / neural networks to create a fetch-playing robot


# 9/15

* The robot has a built in QR code scanner which should enable easy connect. We tried this and the the screen would display success; however, the robot never successfully connected. We attempted this for both a hotspot QR and also the school's network QR. We suspect that the school's QR code might not work since the school's wifi also requires interacting with a webpage to login. We also tried the hotspot at both 5 GHz and 2.4 GHz to maximize compatibility, but neither of them worked. 

# 9/16

* Through and HDMI cable, we were able to finally interact with the robot's OS, which was the Raspberry Pi OS. We were able to successfully connect to TrinityGuest from the terminal. One important note is upon boot the network manager has to be re-enabled. Despite our success in connecting, it seems that we cannot ssh into the robot due to per device isolation by the school's network. We then opted to try using a hotspot. We began setting this up; however, the machine lagged and stopped functioning potentially due to the browser being open in the background. We are not exactly sure why this crash occurred, but it could be due to memory issues. 

# 9/17

* Initially, we believed due to admin updates we would be able to ssh into the server. However, upon trying, ssh connection still failed. After trying to connect to the hotspot again, we noticed that the hotspot was not being detected at all by the robot -- in retrospect, this could have been because the hotspot was not on 2.4 GHz. This led us to try and use Tailscale to connect. Tailscale would enable us to the ssh into the robot and get around the school's network restrictions.

# 9/18
