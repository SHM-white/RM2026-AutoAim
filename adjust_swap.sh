sudo swapoff -a
# 输出当前swap信息
sudo swapon --show
echo "Adjusting swap file to 32GB..."
sudo dd if=/dev/zero of=/swapfile bs=1G count=32
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
echo "Done."
sudo swapon --show
