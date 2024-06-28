import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        # 창의 타이틀 설정
        self.setWindowTitle('Hello PyQt5!')

        # 라벨 위젯 생성 및 텍스트 설정
        self.label = QLabel('Hello, PyQt5!', self)

        # 레이아웃 설정
        layout = QVBoxLayout(self)
        layout.addWidget(self.label)

        # 창의 크기 설정
        self.resize(400, 300)

if __name__ == '__main__':
    app = QApplication(sys.argv)

    # MainWindow 인스턴스 생성 및 표시
    window = MainWindow()
    window.show()

    sys.exit(app.exec_())
