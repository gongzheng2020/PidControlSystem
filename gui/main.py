import sys
from PyQt5.QtWidgets import QApplication,QMainWindow
from stm32plot_ui import Ui_mainWindow
from PyQt5.QtCore import Qt,QTimer
import serial.tools.list_ports  
from serial import Serial
import numpy as np
import pyqtgraph as pg 
from PyQt5.QtGui import QPen 
from pymodbus.client.sync import ModbusSerialClient as ModbusClient
from PyQt5.QtWidgets import QApplication,QStyleFactory

class mainwindow(QMainWindow):
    def __init__(self):
        super().__init__()
        QApplication.setStyle(QStyleFactory.create('Fusion'))
        # 实例化一个 Ui_MainWindow对象
        self.ui=Ui_mainWindow()
        self.setFixedSize(764,480)
        # 设置鼠标点击获取焦点
        self.setFocusPolicy(Qt.NoFocus)
       	# setupUi函数
       	# 这个函数很多地方说是初始化ui对象，我觉得直接翻译为“设置UI”
       	# 这样表明ui对象的实例化和设置（或者说加载）是完全不相干的两步
        self.ui.setupUi(self)
        # 这里使用的是 self.show(),和之后的区分一下
        self.show()
        # 刷新串口号
        ports = serial.tools.list_ports.comports()  
        for port in ports:  
            self.ui.comboBox_5.addItem(port.device)  # 添加串口设备名到下拉列表
        #初始化参数
        self.target=0
        self.Kp=0
        self.Ki=0
        self.Kd=0
        self.timeperiod=50  # ms
        self.slave_id=1
        self.slave_addr=0
        self.slave_sum=6
        # 初始化数据数组  
        self.data = np.array([])  
        self.data2 = np.array([])  # 第二条数据线的数据  

        self.p = self.ui.graphicsView.addPlot()

        # 设置横纵坐标名称  
        self.p.setLabel('bottom', '时间(s)')  # 横坐标，注意units是用来设置坐标轴刻度的单位  
        self.p.setLabel('left', '速度(RPM)')  # 纵坐标  

        # 设置背景颜色
        self.ui.graphicsView.setBackground(pg.mkBrush(255, 255, 255))

        # 创建纯黑色的画笔用于设置刻度颜色  
        black_pen = QPen(pg.mkColor('k'))  # 'k' 代表黑色  
        black_pen.setWidth(2)  # 设置刻度线的宽度  
  
        # 获取x轴和y轴对象，并设置其刻度画笔为黑色  
        self.p.getAxis('bottom').setPen(black_pen)  
        self.p.getAxis('left').setPen(black_pen)  
        self.p.getAxis('left').setTextPen(black_pen) 
        self.p.getAxis('bottom').setTextPen(black_pen) 
        strs = range(0,110,10) # 设置每个刻度值的显示数值
        x = tuple(strs)     # 设置刻度值
        strs = [str(int(i*self.timeperiod*0.002)) for i in strs]
        ticks = [[i, j] for i, j in zip(x,strs)] # 刻度值与显示数值绑定
        self.p.getAxis('bottom').setTicks([ticks])
        self.p.showGrid(True,True)
        self.p.setYRange(0, 80)
        self.p.setXRange(0, 100)

        self.line_plot = self.p.plot(pen=pg.mkPen('r', width=4))   # 红色线条
        self.line_plot2 = self.p.plot(pen=pg.mkPen('b', width=4))  # 蓝色线条  
        # 设置定时器  
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)  
        # 连接信号和槽
        self.ui.doubleSpinBox.editingFinished.connect(self.doubleSpinBox_editingFinished)
        self.ui.doubleSpinBox_2.editingFinished.connect(self.doubleSpinBox_2_editingFinished)
        self.ui.doubleSpinBox_3.editingFinished.connect(self.doubleSpinBox_3_editingFinished)
        self.ui.spinBox_4.editingFinished.connect(self.SpinBox_4_editingFinished)
        self.ui.pushButton.clicked.connect(self.pushButton_clicked)
        self.ui.pushButton_2.clicked.connect(self.pushButton_2_clicked)
        self.ui.comboBox_5.activated.connect(self.comboBox_5_activated)
        self.ui.comboBox_6.activated.connect(self.comboBox_6_activated)

    def update_plot(self):  
        # 读多个寄存器
        response = self.client.read_holding_registers(address=self.slave_addr, count=self.slave_sum, unit=self.slave_id)
        if not response.isError():
            data = response.registers
            print("Register Values: ", data)
        else:
            print("Failed to read registers")
            return

        # 解析数据 
        target =  int(data[0])
        speed = float(data[4]/100)
        Kp = float(data[1]/100)
        Ki = float(data[2]/100)
        Kd = float(data[3]/100)
        self.data = np.append(self.data, target)
        self.data2 = np.append(self.data2, speed)
  
        # 仅保留最近的N个点，以避免数据无限增长  
        self.data = self.data[-100:]
        self.data2 = self.data2[-100:]  
  
        # 更新数据  
        self.line_plot.setData(self.data)
        self.line_plot2.setData(self.data2)
        print(self.target)
        print(target)
        if not self.target == target:
            self.ui.spinBox_4.setValue(target)
        if not self.Kp == Kp:
            self.ui.doubleSpinBox.setValue(Kp)
        if not self.Ki == Ki:
            self.ui.doubleSpinBox_2.setValue(Ki)
        if not self.Kd == Kd:
            self.ui.doubleSpinBox_3.setValue(Kd) 

    def comboBox_6_activated(self):
        self.slave_id=int(self.ui.comboBox_6.currentText())
        if self.client.connect():
            print("Modbus RTU Client Connected")
            response = self.client.read_holding_registers(address=self.slave_addr, count=self.slave_sum, unit=self.slave_id)
            data = response.registers
            self.target =  int(data[0])
            self.Kp = float(data[1]/100)
            self.Ki = float(data[2]/100)
            self.Kd = float(data[3]/100)
            self.ui.spinBox_4.setValue(self.target)
            self.ui.doubleSpinBox.setValue(self.Kp)
            self.ui.doubleSpinBox_2.setValue(self.Ki)
            self.ui.doubleSpinBox_3.setValue(self.Kd) 
        else:
            print("Failed to connect to Modbus RTU Client")
        print(self.slave_id)

    def comboBox_5_activated(self):
        serial_port=self.ui.comboBox_5.currentText()
        # self.serial = Serial(serial_port)  
        # self.serial.baudrate = 115200  
        # self.serial.bytesize = 8  
        # self.serial.parity = 'N'  
        # self.serial.stopbits = 1  
        # self.serial.timeout = 1 
        print(serial_port)
        # 建立连接
        self.client = ModbusClient(method='rtu', port=serial_port, baudrate=115200)
        if self.client.connect():
            print("Modbus RTU Client Connected")
            response = self.client.read_holding_registers(address=self.slave_addr, count=self.slave_sum, unit=self.slave_id)
            data = response.registers
            self.target =  int(data[0])
            self.Kp = float(data[1]/100)
            self.Ki = float(data[2]/100)
            self.Kd = float(data[3]/100)
            self.ui.spinBox_4.setValue(self.target)
            self.ui.doubleSpinBox.setValue(self.Kp)
            self.ui.doubleSpinBox_2.setValue(self.Ki)
            self.ui.doubleSpinBox_3.setValue(self.Kd) 
        else:
            print("Failed to connect to Modbus RTU Client")
        print("comboBox_5_activated")

    def pushButton_2_clicked(self):
        self.timer.stop()
        write_response = self.client.write_register(address=5, value=0, unit=self.slave_id)
        if not write_response.isError():
            print("Written successfully")
        else:
            print("Failed to write register")
        print("pushButton_2_clicked")

    def pushButton_clicked(self):
        self.timer.start(self.timeperiod)
        write_response = self.client.write_register(address=5, value=1, unit=self.slave_id)
        if not write_response.isError():
            print("Written successfully")
        else:
            print("Failed to write register")
        print("pushButton_clicked")

    def doubleSpinBox_editingFinished(self):
        self.Kp=self.ui.doubleSpinBox.value()
        write_response = self.client.write_register(address=1, value=int(self.Kp*100), unit=self.slave_id)
        if not write_response.isError():
            print("Written successfully")
        else:
            print("Failed to write register")
        print("spinBox_editingFinished")

    def doubleSpinBox_2_editingFinished(self):
        self.Ki=self.ui.doubleSpinBox_2.value()
        write_response = self.client.write_register(address=2, value=int(self.Ki*100), unit=self.slave_id)
        if not write_response.isError():
            print("Written successfully")
        else:
            print("Failed to write register")
        print("spinBox2_editingFinished")

    def doubleSpinBox_3_editingFinished(self):
        self.Kd=self.ui.doubleSpinBox_3.value()
        write_response = self.client.write_register(address=3, value=int(self.Kd*100), unit=self.slave_id)
        if not write_response.isError():
            print("Written successfully")
        else:
            print("Failed to write register")
        print("spinBox3_editingFinished")
    
    def SpinBox_4_editingFinished(self):
        self.target=self.ui.spinBox_4.value()
        write_response = self.client.write_register(address=0, value=self.target, unit=self.slave_id)
        if not write_response.isError():
            print("Written successfully")
        else:
            print("Failed to write register")
        print(self.target)
        print("SpinBox_4_editingFinished")

if __name__=="__main__":
    app=QApplication(sys.argv)
    window=mainwindow()
    sys.exit(app.exec_())
