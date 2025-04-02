#!/usr/bin/env python3
import sys
import yaml
from PyQt5.QtWidgets import QApplication, QWidget, QComboBox, QCheckBox,QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QScrollArea, QFrame, QMessageBox, QListWidget, QListWidgetItem
from PyQt5.QtGui import QPalette, QColor, QFont
from PyQt5.QtCore import Qt
import paho.mqtt.client as mqtt
import json
import os
import time

WORKSPACE_PATH = '/home/nvidia/clamp_forklift_ws2'

def run_command(command):
    os.system(f'terminator -e  "{command}" &')
def run_command_restart(command):
    os.system(f'gnome-terminal -e  "{command}" &')
def stop_command(command_name):
    os.system(f'pkill -f "{command_name}"')

class ControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("状态控制参数设置")
        self.setGeometry(400, 300, 900, 650)  # 设置初始窗口大小

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.username_pw_set(username="tju_me", password="tjuME!@#")

        # 尝试连接MQTT，若失败则反复重连
        max_retries = 10
        retry_delay = 5
        for attempt in range(max_retries):
            try:
                self.mqtt_client.connect(host="60.28.24.166", port=1883)
                print("Successfully connected to MQTT broker.")
                break
            except Exception as e:
                print(f"Failed to connect to MQTT broker: {e}. Retrying in {retry_delay} seconds...")
                time.sleep(retry_delay)
        else:
            print("Failed to connect to MQTT broker after multiple attempts.")

        # self.mqtt_client.connect(host="60.28.24.166", port=1883)
        self.SN = "XEIPY30011JA98744"

        self.control_type_combo = QComboBox()
        self.control_type_combo.addItems(["运动控制", "转向控制", "夹抱控制", "升降控制"])
        self.control_type_combo.setStyleSheet("""
            QComboBox {
                background-color: #e6e6e6;
                border: 2px solid #b3b3b3;
                border-radius: 5px;
                padding: 5px;
                font-size: 20px;
                color: #333333;
            }
            QComboBox::drop-down {
                subcontrol-origin: padding;
                subcontrol-position: top right;
                width: 20px;
                border-left-width: 2px;
                border-left-color: #b3b3b3;
                border-left-style: solid;
                border-top-right-radius: 5px;
                border-bottom-right-radius: 5px;
            }
            QComboBox::down-arrow {
                image: url(down_arrow.png);
                width: 10px;
                height: 10px;
            }
            QComboBox QAbstractItemView {
                border: 2px solid #b3b3b3;
                selection-background-color: #5e35b1;
                selection-color: white;
            }
            QComboBox::item {
                background-color: #f0f0f0;
                color: #333333;
            }
            QComboBox::item:selected {
                background-color: #ffffcc;
                color: #333333;
            }
        """)

        self.add_button = QPushButton("添加")
        self.add_button.clicked.connect(self.add_control)
        self.set_button_color(self.add_button, QColor(218, 112, 214))  # #5e35b1
        self.add_button.setStyleSheet("""
            QPushButton {
                background-color: #CF9FFF;
                padding: 8px;
                font-size: 18px;
                color: black;
            }
            QPushButton:hover {
                background-color: #DA70D6;
            }
        """)

        self.generate_button = QPushButton("生成yaml文件")
        self.generate_button.clicked.connect(self.generate_yaml)
        self.set_button_color(self.generate_button, QColor(8, 243, 17))  # #08f311
        self.generate_button.setStyleSheet("""
            QPushButton {
                background-color: #89CFF0;
                padding: 8px;
                font-size: 18px;
                color: #333333;
            }
            QPushButton:hover {
                background-color: #7DF9FF;
            }
        """)

        self.launch_button = QPushButton("启动状态机")
        self.launch_button.clicked.connect(self.launch_state_machine)
        self.set_button_color(self.launch_button, QColor(196, 0, 0))  # #C40000
        self.launch_button.setStyleSheet("""
            QPushButton {
                background-color: #90EE90;
                padding: 8px;
                font-size: 18px;
                color: #333333;
            }
            QPushButton:hover {
                background-color: #08f311;
            }
        """)

        self.stop_button = QPushButton("关闭状态机")
        self.stop_button.clicked.connect(self.stop_state_machine)
        self.set_button_color(self.stop_button, QColor(255, 0, 0))
        self.stop_button.setStyleSheet("""
            QPushButton {
                background-color: #FF3333;
                padding: 8px;
                font-size: 18px;
                color: white;
            }
            QPushButton:hover {
                background-color: #FF0000;
            }
        """)

        self.controls_list = QListWidget()
        self.controls_list.setStyleSheet("""
            QListWidget {
                border: 2px solid #b3b3b3;
                border-radius: 5px;
                padding: 10px;
            }
            QListWidget::item {
                background-color: #f0f0f0;
                border: 2px solid #b3b3b3;
                border-radius: 5px;
            }
            QListWidget::item:hover {
                background-color: #e6e6e6;
            }
        """)
        self.controls_list.setGeometry(10, 100, 580, 200)  # 设置列表框大小

        self.main_layout = QVBoxLayout()

        self.main_layout.addWidget(self.control_type_combo)
        self.main_layout.addWidget(self.add_button)
        self.main_layout.addWidget(self.controls_list)
        self.main_layout.addWidget(self.generate_button)
        self.main_layout.addWidget(self.launch_button)
        self.main_layout.addWidget(self.stop_button)

        author_label = QLabel("Contributed by CYUN")
        author_label.setStyleSheet("""
            QLabel {
                font-size: 18px;
                pedding: 5px;
                color: black;
            }
        """)

        # 创建一个水平布局包裹作者标签，用于居中
        author_layout = QHBoxLayout()
        author_layout.addStretch(1)
        author_layout.addWidget(author_label)
        author_layout.addStretch(1)

        self.main_layout.addLayout(author_layout)

        self.setLayout(self.main_layout)
        self.controls = []

        self.setStyleSheet("""
            QWidget {
                background-color: #f0f0f0;
                color: #333;
            }
            QLineEdit {
                background-color: #fff;
                border: 2px solid #b3b3b3;
                border-radius: 5px;
                font-size: 14px;
            }
            QLabel {
                font-weight: bold;
                font-size: 16px;
                color: #333333;
            }
        """)
    def add_control(self):
        control_type = self.control_type_combo.currentText()
        control_widget = QWidget()
        control_layout = QHBoxLayout(control_widget)  # 把布局关联到这个新的QWidget

        if control_type == "运动控制":
            control_widget.setStyleSheet("background-color: #ffcccc;")
        elif control_type == "转向控制":
            control_widget.setStyleSheet("background-color: #ccffcc;")
        elif control_type == "夹抱控制":
            control_widget.setStyleSheet("background-color: #ccccff;")
        elif control_type == "升降控制":
            control_widget.setStyleSheet("background-color: #ffffcc;")

        control_type_label = QLabel(control_type + ":")
        control_type_label.setAlignment(Qt.AlignLeft)
        control_type_label.setStyleSheet("""
            QLabel {
                font-size: 18px;
                color: #333333;
                padding: 5px;
            }
        """)

        if control_type == "运动控制":
            distance_label = QLabel("distance:")
            distance_entry = QLineEdit()
            speed_label = QLabel("speed:")
            speed_entry = QLineEdit()

            control_layout.addWidget(control_type_label)
            control_layout.addWidget(distance_label)
            control_layout.addWidget(distance_entry)
            control_layout.addWidget(speed_label)
            control_layout.addWidget(speed_entry)

            self.controls.append({
                "type": "motion_control",
                "distance": lambda: float(distance_entry.text()) if distance_entry.text() else 0.0,
                "speed": lambda: float(speed_entry.text()) if speed_entry.text() else 0.0,
                "widget": control_widget
            })

        elif control_type == "转向控制":
            gear_label = QLabel("gear:")
            gear_entry = QLineEdit()
            angle_label = QLabel("angle:")
            angle_entry = QLineEdit()
            speed_label = QLabel("speed:")
            speed_entry = QLineEdit()

            control_layout.addWidget(control_type_label)
            control_layout.addWidget(gear_label)
            control_layout.addWidget(gear_entry)
            control_layout.addWidget(angle_label)
            control_layout.addWidget(angle_entry)
            control_layout.addWidget(speed_label)
            control_layout.addWidget(speed_entry)

            self.controls.append({
                "type": "steering_control",
                "gear": lambda: float(gear_entry.text()) if gear_entry.text() else 0.0,
                "angle": lambda: float(angle_entry.text()) if angle_entry.text() else 0.0,
                "speed": lambda: float(speed_entry.text()) if speed_entry.text() else 0.0,
                "widget": control_widget
            })

        elif control_type == "夹抱控制":
            target_label = QLabel("target:")
            target_entry = QLineEdit()

            control_layout.addWidget(control_type_label)
            control_layout.addWidget(target_label)
            control_layout.addWidget(target_entry)

            self.controls.append({
                "type": "clamp_control",
                "target": lambda: float(target_entry.text()) if target_entry.text() else 0.0,
                "widget": control_widget
            })

        elif control_type == "升降控制":
            target_label = QLabel("target:")
            target_entry = QLineEdit()

            control_layout.addWidget(control_type_label)
            control_layout.addWidget(target_label)
            control_layout.addWidget(target_entry)

            self.controls.append({
                "type": "updown_control",
                "target": lambda: float(target_entry.text()) if target_entry.text() else 0.0,
                "widget": control_widget
            })

        item = QListWidgetItem(self.controls_list)
        item.setSizeHint(control_widget.sizeHint())
        self.controls_list.addItem(item)
        self.controls_list.setItemWidget(item, control_widget)

    def generate_yaml(self):
        # for control in self.controls:
        #     for key, value in control.items():
        #         if key!= "type" and not value():
        #             QMessageBox.warning(self, "警告", "请确保所有字段都已填写！")
        #             return
        for control in self.controls:
            for key, widget in control.items():
                if key != "type":
                    # 根据控件类型检查其值
                    if isinstance(widget, QLineEdit):
                        if not widget.text().strip():  # 检查文本框是否为空
                            QMessageBox.warning(self, "警告", "请确保所有字段都已填写！")
                            return
                    elif isinstance(widget, QComboBox):
                        if widget.currentIndex() == -1:  # 检查下拉列表是否有选择项
                            QMessageBox.warning(self, "警告", "请确保所有字段都已填写！")
                            return
                    elif isinstance(widget, QCheckBox):
                        if not widget.isChecked():  # 检查复选框是否被选中
                            QMessageBox.warning(self, "警告", "请确保所有字段都已填写！")
                            return

        data = []
        for control in self.controls:
            control_data = {}
            if control["type"] == "motion_control":
                control_data["motion_control"] = {
                    "distance": control["distance"](),
                    "speed": control["speed"]()
                }
            elif control["type"] == "steering_control":
                control_data["steering_control"] = {
                    "gear": control["gear"](),
                    "angle": control["angle"](),
                    "speed": control["speed"]()
                }
            elif control["type"] == "clamp_control":
                control_data["clamp_control"] = {
                    "target": control["target"]()
                }
            elif control["type"] == "updown_control":
                control_data["updown_control"] = {
                    "target": control["target"]()
                }
            data.append(control_data)

        with open("/home/nvidia/clamp_forklift_ws2/src/smach_fork/config/output.yaml", 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

    def launch_state_machine(self):
        print("WORKSPACE_PATH: ", WORKSPACE_PATH)
        run_command(f"bash -c 'source {WORKSPACE_PATH}/devel/setup.bash;roslaunch smach_fork smach_fork_custom.launch'")
        message = {
            "cmd_id": "1010",
            "function": "smach_custom",
            "status": "1"
        }
        self.mqtt_client.publish(topic=f"suntae/agv/{self.SN}/task", payload=json.dumps(message), qos=0, retain=False)

    def stop_state_machine(self):
        stop_command("smach_fork_custom.launch")
        message = {
            "cmd_id": "1010",
            "function": "smach_custom",
            "status": "0"
        }
        self.mqtt_client.publish(topic=f"suntae/agv/{self.SN}/task", payload=json.dumps(message), qos=1, retain=False)

    def set_button_color(self, button, color):
        palette = button.palette()
        palette.setColor(QPalette.Button, color)
        button.setPalette(palette)
        button.setAutoFillBackground(True)

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Delete:
            selected_items = self.controls_list.selectedItems()
            for item in selected_items:
                widget = self.controls_list.itemWidget(item)
                for control in self.controls:
                    if control["widget"] == widget:
                        self.controls.remove(control)
                        self.controls_list.takeItem(self.controls_list.row(item))
                        break

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ControlPanel()
    window.show()
    sys.exit(app.exec_())