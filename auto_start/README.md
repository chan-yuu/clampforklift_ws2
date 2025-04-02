只需要给bashrc添加一些内容即可使用完整功能：
export WORKSPACE_PATH="/home/cyun/learn_ws/clamp_forklift_ws"

修改sh中的快捷方式：
forklift_panel.desktop:
Exec=/home/cyun/learn_ws/clamp_forklift_ws/src/auto_start/sh/start_sim_forklift.sh
Terminal=true
Icon=/home/cyun/learn_ws/clamp_forklift_ws/src/auto_start/png/icon.png

然后执行：
sudo cp forklift_panel.desktop ~/.local/share/applications