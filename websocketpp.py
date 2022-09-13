import roslibpy
import roslibpy.tf as tf
from time import sleep
import sys
import base64
import numpy as np
import cv2
import config_record as cr
from robot_visualizer import Ui_robot_visualizer
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QWidget, QInputDialog, QLineEdit
from PyQt5.QtCore import Qt, QThread, QMutex, QTimer, pyqtSignal
from PyQt5.Qt import QPainter, QPixmap, QPen, QPoint, QColor, QImage, QLabel, QIcon, QPointF
from scipy.spatial.transform import Rotation as Rt
from math import sin, cos, atan2


class Thread_1(QThread):  # 线程1

    signal = pyqtSignal(list)

    def __init__(self, func):
        super().__init__()
        self.func = func

    def run(self):
        self.func()


class DrawPaint(QWidget):
    signal = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.data = []
        self.resize(768, 768)
        self.time = QTimer(self)
        self.time.timeout.connect(self.data_list)
        self.color = Qt.blue
        self.scale = 1
        self.point = QPoint(0, 0)
        self.translation_x = 0.0
        self.translation_y = 0.0
        self.rotation_yaw = 0.0
        self.resolution = 0.0
        self.map_axis_x = 0
        self.map_axis_y = 0
        self.base_size = 6
        self.angle = 0.0
        self.pen_color = Qt.green
        self.start_pos = None
        self.end_pos = None
        self.arrow_start_pos = None
        self.arrow_end_pos = None
        self.left_click = False
        self.right_click = False
        self.path_flag = False
        self.paint_move_flag = True
        self.position_draw_flag = False
        self.pose_goal_flag = 0
        self.path_list = []
        self.label = QLabel(self)
        self.label.setStyleSheet('color: rgb(255, 0, 0);')
        self.label.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        # self.setAttribute(Qt.WA_OpaquePaintEvent)

    def paintEvent(self, event):
        # 绘图事件

        paint = QPainter(self)
        # paint.begin(self)
        paint.setPen(self.color)
        paint.scale(self.scale, self.scale)
        if self.data:
            for data_i in self.data:
                if data_i[0] == 0:
                    paint.setPen(Qt.gray)
                    paint.drawPoint(data_i[1] + self.point.x(), data_i[2] + self.point.y())
                else:
                    paint.setPen(Qt.black)
                    paint.drawPoint(data_i[1] + self.point.x(), data_i[2] + self.point.y())
            paint.setPen(Qt.red)
            draw_x = self.point.x() + (self.translation_x / self.resolution)
            draw_y = self.point.y() + (self.translation_y / self.resolution)

            # base_link draw
            # paint.drawEllipse(draw_x + self.map_axis_x - 3, draw_y + self.map_axis_y - 3,
            #                   self.base_size, self.base_size)
            paint.drawEllipse(QPointF(draw_x + self.map_axis_x, draw_y + self.map_axis_y), 3.0, 3.0)
            # x, y axis
            paint.drawLine(QPointF(draw_x + self.map_axis_x, draw_y + self.map_axis_y),
                           QPointF(cos(self.rotation_yaw) * 4 + draw_x + self.map_axis_x,
                                   sin(self.rotation_yaw) * 4 + draw_y + self.map_axis_y))
            paint.drawLine(self.point.x() + self.map_axis_x, self.point.y() + self.map_axis_y,
                           self.point.x() + self.map_axis_x + 10, self.point.y() + self.map_axis_y)
            paint.setPen(Qt.blue)
            paint.drawLine(self.point.x() + self.map_axis_x, self.point.y() + self.map_axis_y,
                           self.point.x() + self.map_axis_x, self.point.y() + self.map_axis_y + 10)
            if self.path_list and self.path_flag:
                paint.setPen(Qt.green)
                for path in self.path_list:
                    paint.drawPoint(QPointF(path['x'] / self.resolution + self.map_axis_x + self.point.x(),
                                    path['y'] / self.resolution + self.map_axis_y + self.point.y()))
        if not self.paint_move_flag or self.position_draw_flag:
            self.draw_arrow(self.arrow_start_pos / self.scale, self.arrow_end_pos / self.scale, paint, self.pen_color)
        paint.setPen(Qt.red)
        paint.drawLine(20, 20, 40, 20)
        paint.drawText(40, 20, 50, 30, 1, 'x')
        paint.setPen(Qt.blue)
        paint.drawLine(20, 20, 20, 0)
        paint.drawText(10, 0, 20, 20, 1, 'y')
        # paint.end()

    def wheelEvent(self, event):
        angle = event.angleDelta() / 8  # 返回QPoint对象，为滚轮转过的数值，单位为1/8度
        angle_y = angle.y()
        if angle_y > 0:
            self.scale *= 1.1
        else:  # 滚轮下滚
            self.scale *= 0.9
        self.adjustSize()
        self.update()

    def mouseMoveEvent(self, e):
        """
        mouse move events for the widget
        :param e: QMouseEvent
        :return:
        """
        if self.paint_move_flag:
            if self.left_click:
                self.end_pos = (e.pos() - self.start_pos) / self.scale
                self.point = self.point + self.end_pos
                self.start_pos = e.pos()
                self.update()
                # self.repaint()
            elif self.right_click:
                self.label.move(e.pos().x() + 20, e.pos().y() - 20)
                self.label.setText('x = ' + str(round((e.pos().x() / self.scale - self.point.x() - self.map_axis_x) / 20.0, 3)) +
                                   '\n' + 'y = ' + str(
                    round((e.pos().y() / self.scale - self.point.y() - self.map_axis_y) / 20.0, 3)))
        else:
            self.arrow_end_pos = e.pos()
            self.update()

    def mousePressEvent(self, e):
        """
        mouse press events for the widget
        :param e: QMouseEvent
        :return:
        """
        if self.paint_move_flag:
            if e.button() == Qt.LeftButton:
                self.left_click = True
                self.start_pos = e.pos()
            elif e.button() == Qt.RightButton:
                self.right_click = True
                self.label.move(e.pos().x() + 20, e.pos().y() - 20)
                self.label.setText('x = ' + str(round((e.pos().x() / self.scale - self.point.x() - self.map_axis_x) / 20.0, 3)) +
                                   '\n' + 'y = ' + str(
                    round((e.pos().y() / self.scale - self.point.y() - self.map_axis_y) / 20.0, 3)))
                self.label.show()
        else:
            self.arrow_start_pos = e.pos()

    def mouseReleaseEvent(self, e):
        """
        mouse release events for the widget
        :param e: QMouseEvent
        :return:
        """
        if self.paint_move_flag:
            if e.button() == Qt.LeftButton:
                self.left_click = False
            elif e.button() == Qt.RightButton:
                self.right_click = False
                self.label.close()
        else:
            self.paint_move_flag = True
            self.signal.emit(self.pose_goal_flag)
            self.pose_goal_flag = 0

    def data_list(self):
        self.update()

    def draw_arrow(self, begin, end, paint, color):
        x1 = begin.x()
        y1 = begin.y()
        x2 = end.x()
        y2 = end.y()

        self.angle = atan2(y2 - y1, x2 - x1)
        len_arrow = 5
        angle_arrow = 0.5
        new_x1 = x2 - len_arrow * cos(self.angle - angle_arrow)
        new_y1 = y2 - len_arrow * sin(self.angle - angle_arrow)
        new_x2 = x2 - len_arrow * cos(self.angle + angle_arrow)
        new_y2 = y2 - len_arrow * sin(self.angle + angle_arrow)

        paint.setPen(QPen(color, 2))
        paint.drawLine(begin, end)
        paint.drawLine(end, QPointF(new_x1, new_y1))
        paint.drawLine(end, QPointF(new_x2, new_y2))


class robot_display(QMainWindow, Ui_robot_visualizer):

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowIcon(QIcon('logo.ico'))
        self.control_object_init()
        self.showMaximized()
        self.lineEdit_ip.setText(cr.config_read(1))
        self.lineEdit_port.setText(cr.config_read(2))
        self.host = self.lineEdit_ip.text()
        self.port = self.lineEdit_port.text()
        self.groupBox_control.setEnabled(False)
        self.groupBox_coordinate.setVisible(False)
        self.groupBox_data_reset.setVisible(False)
        self.lineEdit_px.setText('1.638')
        self.lineEdit_py.setText('-0.658')
        self.lineEdit_ox.setText('-0.002')
        self.lineEdit_oy.setText('0.003')
        self.lineEdit_oz.setText('-0.701')
        self.lineEdit_ow.setText('0.714')
        self.pub_flag = True
        self.connect_flag = 0
        self.linear = 0.0
        self.angular = 0.0
        self.push_con_list = [self.pushButton_q, self.pushButton_w, self.pushButton_e,
                              self.pushButton_a, self.pushButton_s, self.pushButton_d,
                              self.pushButton_z, self.pushButton_x, self.pushButton_c]
        self.key_list = [Qt.Key_Q, Qt.Key_W, Qt.Key_E,
                         Qt.Key_A, Qt.Key_S, Qt.Key_D,
                         Qt.Key_Z, Qt.Key_X, Qt.Key_C]
        self.key_msg = ['q', 'w', 'e', 'a', 's', 'd', 'z', 'x', 'c']

        self.client = roslibpy.Ros(host=self.host, port=int(self.port))
        # topic
        self.pub_cmd = roslibpy.Topic(self.client, '/cmd_vel', 'geometry_msgs/Twist')
        self.sub_map = roslibpy.Topic(self.client, '/map', 'nav_msgs/OccupancyGrid')
        self.sub_cam = roslibpy.Topic(self.client, '/usb_cam/image_raw/compressed', 'sensor_msgs/CompressedImage')
        self.sub_scan = roslibpy.Topic(self.client, '/scan_filtered', 'sensor_msgs/LaserScan')
        self.stop_status = roslibpy.Topic(self.client, '/stop_status', 'std_msgs/String')
        self.send_goal = roslibpy.Topic(self.client, '/move_base_simple/goal', 'geometry_msgs/PoseStamped')
        self.send_pose_2d = roslibpy.Topic(self.client, '/initialpose', 'geometry_msgs/PoseWithCovarianceStamped')

        # service
        self.battery_srv = roslibpy.Service(self.client, 'robot_check_srv', 'serial_demo/RobotSelfCheck')
        self.go_home_srv = roslibpy.Service(self.client, 'robot_go_home', 'serial_demo/GoHome')

        # TF
        self.tf_client = tf.TFClient(self.client, '/map', angular_threshold=0.3)
        self.pushButton_go_home.setEnabled(int(cr.config_read(3)))
        self.time_srv = QTimer()
        self.reset_timer = QTimer()
        self.widget_map1 = DrawPaint()
        self.connect_t = Thread_1(self.connect_robot)
        self.srv_connect = Thread_1(self.robot_go_home_callback)
        self.gridLayout_5.addWidget(self.widget_map1, 1, 0, 2, 7)
        with open('message.txt', 'r', encoding='utf-8') as f:
            text = f.read()
            self.textBrowser_notice.setText(text)

        self.signal_init()
        # self.test()

    def control_object_init(self):
        # self.pushButton_2d_goal.setStyleSheet('border:none')
        # self.pushButton_2d_pose.setStyleSheet('border:none')
        self.pushButton_2d_goal.setIcon(QIcon('icon/SetGoal.png'))
        self.pushButton_2d_pose.setIcon(QIcon('icon/SetInitialPose.png'))
        self.pushButton_record_path.setIcon(QIcon('icon/Path.png'))
        self.pushButton_reset_path.setIcon(QApplication.style().standardIcon(46))
        self.pushButton_reset_local.setIcon(QIcon('icon/PublishPoint.svg'))
        self.pushButton_go_home.setIcon(QIcon('icon/battery.png'))

    def signal_init(self):
        self.pushButton_connect.clicked.connect(self.connect_robot)
        self.pushButton_pub.clicked.connect(self.publish_cmd)
        self.pushButton_sub.clicked.connect(self.sub_msg)
        self.pushButton_go_home.clicked.connect(self.connect_signal)
        self.pushButton_open.clicked.connect(self.open_camera)
        self.pushButton_close.clicked.connect(self.close_camera)
        self.pushButton_reset_local.clicked.connect(self.reset_local_callback)
        self.pushButton_points_go.clicked.connect(self.send_goal_callback)
        self.pushButton_record_path.clicked.connect(self.path_callback)
        self.pushButton_reset_path.clicked.connect(self.path_reset)
        self.pushButton_2d_pose.clicked.connect(self.pose_2d_callback)
        self.pushButton_2d_goal.clicked.connect(self.goal_2d_callback)
        self.horizontalSlider_speed.valueChanged.connect(self.speed_set)
        self.time_srv.timeout.connect(self.battery_srv_callback)
        self.stop_status.subscribe(self.stop_go_home)
        self.comboBox_points.currentIndexChanged.connect(self.coordinate_show_callback)
        self.reset_timer.timeout.connect(self.reset_speed_callback)
        self.checkBox_reset.clicked.connect(self.data_reset_callback)
        self.widget_map1.signal.connect(self.pose_goal_message_callback)
        self.lineEdit_ip.textChanged.connect(self.ros_client_init)
        self.action_battery.triggered.connect(self.action_battery_callback)

    def keyPressEvent(self, event):
        if event.key() in self.key_list and self.groupBox_control.isEnabled():
            index = self.key_list.index(event.key())
            self.push_con_list[index].setStyleSheet('background-color: rgb(133, 133, 255);')
            self.speed_set()
            self.twist_pub(self.key_msg[index])

    def keyReleaseEvent(self, event):
        if event.key() in self.key_list and self.groupBox_control.isEnabled():
            index = self.key_list.index(event.key())
            self.push_con_list[index].setStyleSheet('background-color: rgb(227, 227, 227);')

    def connect_signal(self):
        self.srv_connect.start()

    def ros_client_init(self):
        self.client = roslibpy.Ros(host=self.lineEdit_ip.text(), port=int(self.port))
        # topic
        self.pub_cmd = roslibpy.Topic(self.client, '/cmd_vel', 'geometry_msgs/Twist')
        self.sub_map = roslibpy.Topic(self.client, '/map', 'nav_msgs/OccupancyGrid')
        self.sub_cam = roslibpy.Topic(self.client, '/usb_cam/image_raw/compressed', 'sensor_msgs/CompressedImage')
        self.sub_scan = roslibpy.Topic(self.client, '/scan_filtered', 'sensor_msgs/LaserScan')
        self.stop_status = roslibpy.Topic(self.client, '/stop_status', 'std_msgs/String')
        self.send_goal = roslibpy.Topic(self.client, '/move_base_simple/goal', 'geometry_msgs/PoseStamped')
        self.send_pose_2d = roslibpy.Topic(self.client, '/initialpose', 'geometry_msgs/PoseWithCovarianceStamped')

        # service
        self.battery_srv = roslibpy.Service(self.client, 'robot_check_srv', 'serial_demo/RobotSelfCheck')
        self.go_home_srv = roslibpy.Service(self.client, 'robot_go_home', 'serial_demo/GoHome')

        # TF
        self.tf_client = tf.TFClient(self.client, '/map', angular_threshold=0.3)

        # config_write
        cr.config.set('websocket_message', 'ip', self.lineEdit_ip.text())
        cr.config.set('websocket_message', 'port', self.lineEdit_port.text())
        cr.config.write(open('config.ini', 'w'))

    def connect_robot(self):
        if self.connect_flag == 0:
            self.pushButton_connect.setText('断开连接')
            self.connect_flag = 1
            self.log_show('连接中...')
            try:
                self.client.run(timeout=5)
                self.log_show('连接成功！')
                self.battery_srv_callback()
                self.time_srv.start(10000)
            except Exception as e:
                print(e)
                self.connect_flag = 0
                self.pushButton_connect.setText('重新连接')
                self.log_show('连接超时,检查网络状态后重试！')
        elif self.connect_flag == 1:
            self.pushButton_connect.setText('重新连接')
            self.client.close()
            self.time_srv.stop()
            self.connect_flag = 2

        elif self.connect_flag == 2:
            self.client.connect()
            self.log_show('重连成功！')
            self.pushButton_connect.setText('断开连接')
            self.battery_srv_callback()
            self.time_srv.start(10000)
            self.connect_flag = 1

    def publish_cmd(self):
        if self.pub_flag:
            self.groupBox_control.setEnabled(True)
            self.pushButton_pub.setText('暂停')
            self.pub_flag = False
        else:
            self.groupBox_control.setEnabled(False)
            self.pushButton_pub.setText('发布')
            self.pub_flag = True

    def twist_pub(self, key):
        if self.client.is_connected:
            if key == 'w':
                self.pub_cmd.publish(roslibpy.Message({'linear': {'x': self.linear}, 'angular': {'z': 0.0}}))
            elif key == 'a':
                self.pub_cmd.publish(roslibpy.Message({'linear': {'x': 0.0}, 'angular': {'z': self.angular}}))
            elif key == 'd':
                self.pub_cmd.publish(roslibpy.Message({'linear': {'x': 0.0}, 'angular': {'z': - self.angular}}))
            elif key == 'x':
                self.pub_cmd.publish(roslibpy.Message({'linear': {'x': - self.linear}, 'angular': {'z': 0.0}}))
            elif key == 'q':
                self.pub_cmd.publish(roslibpy.Message({'linear': {'x': self.linear}, 'angular': {'z': self.angular}}))
            elif key == 'e':
                self.pub_cmd.publish(roslibpy.Message({'linear': {'x': self.linear}, 'angular': {'z': - self.angular}}))
            elif key == 'z':
                self.pub_cmd.publish(
                    roslibpy.Message({'linear': {'x': - self.linear}, 'angular': {'z': - self.angular}}))
            elif key == 'c':
                self.pub_cmd.publish(roslibpy.Message({'linear': {'x': - self.linear}, 'angular': {'z': self.angular}}))
            elif key == 's':
                self.pub_cmd.publish(roslibpy.Message({'linear': {'x': 0.0}, 'angular': {'z': 0.0}}))

    def speed_set(self):
        self.linear = 0.5 * self.horizontalSlider_speed.value() / 100.0
        self.angular = self.linear * 2.0
        self.label_linear.setText(str(self.linear) + 'm/s')
        self.label_angular.setText(str(self.angular) + 'rad/s')

    def sub_msg(self):
        self.sub_map.subscribe(lambda msg: self.map_callback(msg))
        # self.sub_scan.subscribe(lambda msg: self.scan_callback(msg))
        self.tf_client.subscribe('/base_link', self.tf_position)

    def tf_position(self, msg):
        self.widget_map1.translation_x = msg['translation']['x']
        self.widget_map1.translation_y = msg['translation']['y']
        self.widget_map1.path_list.append(msg['translation'])
        quat = [msg['rotation']['x'], msg['rotation']['y'], msg['rotation']['z'], msg['rotation']['w']]
        rq = Rt.from_quat(quat)
        self.widget_map1.rotation_yaw = rq.as_euler('xyz', degrees=False).tolist()[2]
        self.widget_map1.data_list()

    def path_callback(self):
        if self.widget_map1.path_flag:
            self.widget_map1.path_flag = False
            self.pushButton_record_path.setText('记录轨迹')
        else:
            self.widget_map1.path_flag = True
            self.pushButton_record_path.setText('停止记录')
        self.widget_map1.data_list()

    def path_reset(self):
        self.widget_map1.path_list.clear()
        self.widget_map1.data_list()
        self.log_show('轨迹清空完成！')

    def stop_go_home(self, msg):
        self.pushButton_go_home.setText('回充')
        self.log_show('回充完成，机器充电中...')

    def scan_callback(self, msg):
        print(msg['intensities'])

    def map_callback(self, msg):
        origin_x = - msg['info']['origin']['position']['x']
        origin_y = - msg['info']['origin']['position']['y']
        self.widget_map1.resolution = round(msg['info']['resolution'], 3)
        self.widget_map1.map_axis_x = int(origin_x / self.widget_map1.resolution)
        self.widget_map1.map_axis_y = int(origin_y / self.widget_map1.resolution)
        width = msg['info']['width']
        data = msg['data']
        data_list = []
        x_index = 0
        y_index = 0
        for i in range(len(data)):
            if data[i] == -1:
                pass
            elif data[i] == 0:
                a = [0, x_index, y_index]
                data_list.append(a)
            else:
                a = [100, x_index, y_index]
                data_list.append(a)

            x_index += 1
            if x_index == width:
                y_index += 1
                x_index = 0
        self.widget_map1.data = data_list
        self.widget_map1.update()
        self.sub_map.unsubscribe()

    def map_draw(self):
        pass

    def battery_srv_callback(self):
        request = roslibpy.ServiceRequest()
        request.data = {'command': 'check'}
        result = self.battery_srv.call(request)
        self.progressBar_battery.setValue(result['battery'])

    def robot_go_home_callback(self):
        self.pushButton_go_home.setText('回充中...')
        request_go_home = roslibpy.ServiceRequest()
        request_go_home.data = {'command': 'go',
                                'origin': {'position': {'x': float(self.lineEdit_px.text()),
                                                        'y': float(self.lineEdit_py.text())},
                                           'orientation': {'x': float(self.lineEdit_ox.text()),
                                                           'y': float(self.lineEdit_oy.text()),
                                                           'z': float(self.lineEdit_oz.text()),
                                                           'w': float(self.lineEdit_ow.text())}}}
        result = self.go_home_srv.call(request_go_home)
        if result['go_home'] == 1:
            # self.pushButton_go_home.setText('回充中...')
            self.log_show('到达回充点！')

    def open_camera(self):
        self.sub_cam.subscribe(self.show_camera)

    def close_camera(self):
        self.sub_cam.unsubscribe()
        self.label_camera.setPixmap(QPixmap(''))

    def show_camera(self, msg):
        data_dec = base64.b64decode(msg['data'].encode('ascii'))
        buffer = np.frombuffer(data_dec, dtype=np.uint8)
        image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        video = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        show_video = QImage(video.data, video.shape[1], video.shape[0], QImage.Format_RGB888)
        self.label_camera.setPixmap(QPixmap.fromImage(show_video))

    def log_show(self, msg):
        self.listWidget_log.addItem(msg)
        self.listWidget_log.scrollToBottom()

    def coordinate_show_callback(self):
        if self.comboBox_points.currentIndex() == 3:
            self.groupBox_coordinate.setVisible(True)
        else:
            self.groupBox_coordinate.setVisible(False)

    def reset_local_callback(self):
        self.log_show('正在执行重定位！请勿执行其他操作！')
        self.reset_flag = 0
        if self.horizontalSlider_speed.value() == 0:
            self.angular = 0.3
        self.reset_timer.start(500)

    def reset_speed_callback(self):
        if self.reset_flag < 10:
            self.twist_pub('a')
            self.reset_flag += 1
        elif 10 <= self.reset_flag < 30:
            self.twist_pub('d')
            self.reset_flag += 1
        elif 30 <= self.reset_flag < 40:
            self.twist_pub('a')
            self.reset_flag += 1
        else:
            self.reset_timer.stop()
            self.angular = 0.0
            self.log_show('重定位完成！')

    def send_goal_callback(self):
        goal_position_x = 0.0
        goal_position_y = 0.0
        goal_orientation_x = 0.0
        goal_orientation_y = 0.0
        goal_orientation_z = 0.0
        goal_orientation_w = 1.0
        if self.comboBox_points.currentIndex() == 1:
            self.log_show('正在导航回充点！')
            goal_position_x = 2.284
            goal_position_y = 1.156
            goal_orientation_x = -0.002
            goal_orientation_y = -0.002
            goal_orientation_z = 0.776
            goal_orientation_w = 0.631
        elif self.comboBox_points.currentIndex() == 2:
            self.log_show('正在导航测试点！')
            goal_position_x = 4.105
            goal_position_y = 0.797
            goal_orientation_x = -0.000
            goal_orientation_y = 0.003
            goal_orientation_z = 0.998
            goal_orientation_w = -0.069
        elif self.comboBox_points.currentIndex() == 3:
            self.log_show('正在导航自定义点！')
            goal_position_x = float(self.lineEdit_x.text())
            goal_position_y = float(self.lineEdit_y.text())

            degrees = Rt.from_euler('xyz', [0.0, 0.0, float(self.lineEdit_yaw.text())], degrees=True)
            qua = degrees.as_quat().tolist()
            goal_orientation_x = qua[0]
            goal_orientation_y = qua[1]
            goal_orientation_z = qua[2]
            goal_orientation_w = qua[3]
        self.send_goal.publish(roslibpy.Message({'header': {'frame_id': 'map'},
                                                 'pose': {'position': {'x': goal_position_x,
                                                                       'y': goal_position_y},
                                                          'orientation': {'x': goal_orientation_x,
                                                                          'y': goal_orientation_y,
                                                                          'z': goal_orientation_z,
                                                                          'w': goal_orientation_w}}}))

    def data_reset_callback(self):
        if self.checkBox_reset.isChecked():
            self.groupBox_data_reset.setVisible(True)
        else:
            self.groupBox_data_reset.setVisible(False)

    def pose_2d_callback(self):
        self.widget_map1.paint_move_flag = False
        self.widget_map1.pen_color = Qt.green
        self.widget_map1.pose_goal_flag = 1

    def goal_2d_callback(self):
        self.widget_map1.paint_move_flag = False
        self.widget_map1.pen_color = QColor(255, 87, 233)
        self.widget_map1.pose_goal_flag = 2

    def pose_goal_message_callback(self, msg):
        goal_position_x = self.widget_map1.arrow_start_pos.x() / self.widget_map1.scale - self.widget_map1.point.x() - self.widget_map1.map_axis_x
        goal_position_y = self.widget_map1.arrow_start_pos.y() / self.widget_map1.scale - self.widget_map1.point.y() - self.widget_map1.map_axis_y
        degrees = Rt.from_euler('xyz', [0.0, 0.0, self.widget_map1.angle], degrees=False)
        qua = degrees.as_quat().tolist()
        goal_orientation_x = qua[0]
        goal_orientation_y = qua[1]
        goal_orientation_z = qua[2]
        goal_orientation_w = qua[3]
        if msg == 1:
            self.log_show('正在使用2D_pose指定位姿...')
            covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          0.06853892326654787]
            self.send_pose_2d.publish(roslibpy.Message({'header': {'frame_id': 'map'},
                                                        'pose': {'pose': {'position': {'x': goal_position_x / 20.0,
                                                                                       'y': goal_position_y / 20.0},
                                                                          'orientation': {'x': goal_orientation_x,
                                                                                          'y': goal_orientation_y,
                                                                                          'z': goal_orientation_z,
                                                                                          'w': goal_orientation_w}},
                                                                 'covariance': covariance}}))

        elif msg == 2:
            self.log_show('正在使用2D_goal导航中...')
            self.send_goal.publish(roslibpy.Message({'header': {'frame_id': 'map'},
                                                     'pose': {'position': {'x': goal_position_x / 20.0,
                                                                           'y': goal_position_y / 20.0},
                                                              'orientation': {'x': goal_orientation_x,
                                                                              'y': goal_orientation_y,
                                                                              'z': goal_orientation_z,
                                                                              'w': goal_orientation_w}}}))

    def action_battery_callback(self):
        value, ok = QInputDialog.getText(self, '警告', '请输入管理员密码', QLineEdit.Password)
        if value == '123':
            self.pushButton_go_home.setEnabled(True)
            QMessageBox.information(self, '信息', '管理员密码验证成功!')
        else:
            QMessageBox.warning(self, '警告', '管理员密码验证失败!')

    def test(self):
        import pyqtgraph as pg
        import pyqtgraph.opengl as gl

        gvw = gl.GLViewWidget()

        griditem = gl.GLGridItem()
        griditem.setSize(10, 10)
        griditem.setSpacing(0.5, 0.5)
        gvw.addItem(griditem)

        axisitem = gl.GLAxisItem()
        axisitem.setSize(10, 10, 10)
        gvw.addItem(axisitem)
        txtitem1 = gl.GLTextItem(pos=(0.0, 0.0, 0.0), text='map')
        gvw.addItem(txtitem1)

        txtitem2 = gl.GLTextItem()
        txtitem2.setData(pos=(1.0, -1.0, 2.0), color=(127, 255, 127, 255), text='odom')
        gvw.addItem(txtitem2)
        self.gridLayout_5.addWidget(gvw, 1, 0, 1, 7)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = robot_display()
    ui.show()
    sys.exit(app.exec_())
