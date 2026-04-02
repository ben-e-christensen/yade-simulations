# 1. Add the repository
sudo bash -c 'echo "deb http://www.yade-dem.org/packages/ $(lsb_release -cs) main" > /etc/apt/sources.list.d/yadedaily.list'

# 2. Add the GPG key using the updated, supported method
wget -qO - http://www.yade-dem.org/packages/yadedev_pub.gpg | sudo tee /etc/apt/trusted.gpg.d/yadedaily.asc > /dev/null

# 3. Update your package list and install
sudo apt-get update
sudo apt-get install yadedaily