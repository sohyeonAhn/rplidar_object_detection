from PyQt5.QtCore import Qt, QPoint, pyqtSignal
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QBrush, QColor
import math

class Joystick(QWidget):
    # Signal to emit the precise coordinates as a tuple
    positionChanged = pyqtSignal(tuple)

    def __init__(self, parent=None):
        super(Joystick, self).__init__(parent)
        self.setMinimumSize(200, 200)  # Set the minimum size of the widget
        self.setMaximumSize(200, 200)  # Set the maximum size of the widget
        self.joystick_radius = 80  # Set the radius of the joystick
        self.center = QPoint(100, 100)  # Set the position of the joystick's center
        self.position = self.center  # Initialize the current position of the joystick
        self.dragging = False  # Flag to indicate if the joystick is being dragged

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)  # Enable antialiasing for smoother drawing
        painter.setBrush(QBrush(QColor(200, 200, 200), Qt.SolidPattern))  # Set color and pattern for the outer circle
        painter.drawEllipse(self.center, self.joystick_radius, self.joystick_radius)  # Draw the outer circle
        painter.setBrush(QBrush(QColor(100, 100, 100), Qt.SolidPattern))  # Set color and pattern for the inner circle
        painter.drawEllipse(self.position, 30, 30)  # Draw the inner circle

    def mousePressEvent(self, event):
        if (event.pos() - self.center).manhattanLength() < self.joystick_radius:
            self.dragging = True  # Start dragging
            self.updatePosition(event.pos())  # Update position

    def mouseMoveEvent(self, event):
        if self.dragging:
            self.updatePosition(event.pos())  # Update position while dragging

    def mouseReleaseEvent(self, event):
        self.dragging = False  # Stop dragging
        self.updatePosition(self.center)  # Reset joystick to center

    def updatePosition(self, pos):
        try:
            delta = pos - self.center
            distance = delta.manhattanLength()
            if distance <= self.joystick_radius:
                self.position = pos
            else:
                angle = math.radians(delta.angle())
                x = self.joystick_radius * math.cos(angle)
                y = self.joystick_radius * math.sin(angle)
                self.position = self.center + QPoint(int(x), int(y))

            # Normalize the position to range from -1 to 1 with 0.1 steps
            normalized_x = -(self.position.x() - self.center.x()) / self.joystick_radius  # Invert x-axis
            normalized_y = (self.center.y() - self.position.y()) / self.joystick_radius  # Invert y-axis
            normalized_x = round(10 * normalized_x) / 10  # Round to nearest 0.1
            normalized_y = round(10 * normalized_y) / 10  # Round to nearest 0.1
            normalized_tuple = (normalized_x, normalized_y)
            self.positionChanged.emit(normalized_tuple)  # Emit the position change signal
            print(f"Joystick position: {normalized_tuple}")  # Output the current position
            self.update()  # Update the widget
        except Exception as e:
            print(f"Error: {str(e)}")  # Print errors to the console
