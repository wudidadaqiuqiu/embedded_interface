import sys
from PyQt5.QtCore import Qt, QThread
from PyQt5.QtGui import QPainter, QPen, QColor, QVector2D
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QGraphicsScene, QGraphicsView
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Float32

v1 = np.array([0, -1])
v2 = np.array([math.sqrt(3)/2, 1/2])
v3 = np.array([-math.sqrt(3)/2, 1/2])


def node_init(args=None):
    rclpy.init(args=args)
    n = Node("key_pub")
    return n


def main(node: Node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class RosThread(QThread):
    def __init__(self):
        super().__init__()
        self.node = node_init()
    def run(self):
        main(self.node)

class VectorEditor(QWidget):
    def __init__(self):
        super().__init__()
        
        self.vector = [0, 0]  # 初始向量 (x, y)
        self.init_ui()
        

    def init_ui(self):
        self.setWindowTitle('Vector Visualizer')

        # 显示向量的标签
        self.vector_label = QLabel(f"Vector: ({self.vector[0]}, {self.vector[1]})", self)
        self.vector_label.setAlignment(Qt.AlignCenter)

        # 设置场景和视图
        self.scene = QGraphicsScene()
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing)  # 开启抗锯齿

        # 创建布局
        layout = QVBoxLayout()
        layout.addWidget(self.vector_label)
        layout.addWidget(self.view)

        self.setLayout(layout)
        self.setGeometry(100, 100, 600, 400)

        self.update_visualization()  # 初始化显示向量
        self.ros = RosThread()
        self.ros.pub = self.ros.node.create_publisher(Float32MultiArray, 'vel3', 10)
        self.msg = Float32MultiArray()
        self.ros.start()

    def keyPressEvent(self, event):
        """重写键盘事件"""
        if event.key() == Qt.Key_A:
            self.vector[1] = 10  # y 方向加 1
        elif event.key() == Qt.Key_D:
            self.vector[1] = -10  # y 方向减 1
        elif event.key() == Qt.Key_W:
            self.vector[0] = 10  # x 方向加 1
        elif event.key() == Qt.Key_S:
            self.vector[0] = -10  # x 方向减 1
        elif event.key() == Qt.Key_Q:
            self.vector = [0, 0]  # 重置向量

        # 更新显示的向量
        self.vector_label.setText(f"Vector: ({self.vector[0]}, {self.vector[1]})")
        # print([type(np.array(self.vector).dot(v1)), np.array(self.vector).dot(v2), np.array(self.vector).dot(v3)])
        self.msg.data = [float(np.array(self.vector).dot(v1)), float(np.array(self.vector).dot(v2)),
                        float(np.array(self.vector).dot(v3))]
        for i in range(len(self.msg.data)):
            self.msg.data[i] *= 10 
        self.ros.pub.publish(self.msg)
        print(self.msg)
        # print(self.vector)
        self.update_visualization()

    def update_visualization(self):
        """更新向量的可视化"""
        self.scene.clear()  # 清空场景

        # 创建画笔
        pen = QPen(QColor(255, 0, 0))  # 红色的箭头
        pen.setWidth(3)

        # 创建起点和终点
        origin = QVector2D(0, 0)  # 原点
        end = QVector2D(-self.vector[1], -self.vector[0])  # 向量终点 (反转y轴)

        # 绘制箭头（线段和箭头头）
        self.scene.addLine(origin.x(), origin.y(), end.x(), end.y(), pen)

        # 绘制箭头的箭头头部
        arrow_size = 10
        angle = end - origin
        angle.normalize()

        # 绘制箭头头部（两个小线段）
        self.scene.addLine(end.x(), end.y(), end.x() - angle.y() * arrow_size, end.y() + angle.x() * arrow_size, pen)
        self.scene.addLine(end.x(), end.y(), end.x() + angle.y() * arrow_size, end.y() - angle.x() * arrow_size, pen)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = VectorEditor()
    window.show()
    sys.exit(app.exec_())
