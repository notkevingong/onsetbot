# volleyball_gui.py
import rclpy
from rclpy.node import Node
from onset_interfaces.msg import LaunchCommand, OnsetStatus
import sys
import math
import time
import signal
from PyQt6.QtCore import Qt, QPointF, QTimer
from PyQt6.QtGui import QPen, QBrush, QPainter, QPainterPath
from PyQt6.QtWidgets import (
    QApplication, QGraphicsRectItem, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QGraphicsView, QGraphicsScene, QGraphicsEllipseItem, QGraphicsLineItem,
    QGraphicsTextItem, QGraphicsPathItem, QPushButton
)

# Global variable to store current state
setter_state = {
    'ball_x': 0.0,
    'ball_z': 0.0,
    'ball_y': 0.0,
    'parabola_peak': 0.0,
    'initial_velocity': 0.0,
    'launch_angle': 0.0,
    'yaw_angle': 0.0
}

class OnsetbotGuiRosNode(Node):
    def __init__(self):
        super().__init__('onsetbot_gui_node')
        self.cmd_pub = self.create_publisher(LaunchCommand, '/launch_info', 10)
        self.onset_is_homed = 0
        self.onset_is_busy = 0
        self._prev_onset_is_busy = 0
        self._home_request_pending = False
        self._home_request_reset_timer = None
        self._home_request_debounce_s = 0.35
        self._home_request_reset_delay_s = 0.2
        self._last_home_request_ts = 0.0
        self.status_sub = self.create_subscription(
            OnsetStatus,
            '/onset_status',
            self.onset_status_callback,
            10
        )

    def publish_odrive_command(self, velocity: float, angle_turret: float, angle_launch: float):
        msg = LaunchCommand()
        msg.velocity = float(velocity)
        msg.angle_turret = float(angle_turret)
        msg.angle_launch = float(angle_launch)
        self.cmd_pub.publish(msg)
        # Optional debug log (can be noisy while dragging)
        # self.get_logger().info(
        #     f'Published LaunchCommand: velocity={msg.velocity:.3f}, angle_turret={msg.angle_turret:.3f}'
        # )

    def onset_status_callback(self, msg: OnsetStatus):
        self.onset_is_homed = int(msg.onset_is_homed)
        self.onset_is_busy = int(msg.onset_is_busy)

        busy_rising = (self._prev_onset_is_busy == 0) and (self.onset_is_busy == 1)
        if self._home_request_pending and busy_rising:
            self._home_request_pending = False
            self._schedule_home_request_reset(self._home_request_reset_delay_s)

        self._prev_onset_is_busy = self.onset_is_busy

    def can_launch(self) -> bool:
        return (self.onset_is_homed == 1) and (self.onset_is_busy == 0)

    def publish_home_onset_request(self, request_value: int):
        msg = LaunchCommand()
        msg.home_onset_request = int(request_value)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Published home_onset_request={msg.home_onset_request} to /launch_info')

    def _schedule_home_request_reset(self, delay_s: float):
        if self._home_request_reset_timer is not None:
            self._home_request_reset_timer.cancel()
            self._home_request_reset_timer = None

        def _reset_request():
            self._home_request_pending = False
            self.publish_home_onset_request(0)
            if self._home_request_reset_timer is not None:
                self._home_request_reset_timer.cancel()
                self._home_request_reset_timer = None

        self._home_request_reset_timer = self.create_timer(delay_s, _reset_request)

    def request_home_onset(self):
        now = time.monotonic()

        if self._home_request_pending:
            self.get_logger().info('Ignoring home request: already pending')
            return

        if (now - self._last_home_request_ts) < self._home_request_debounce_s:
            self.get_logger().info('Ignoring home request: debounced')
            return

        self.publish_home_onset_request(1)
        self._last_home_request_ts = now
        self._home_request_pending = True

        # Failsafe reset in case acknowledgement status is missed
        self._schedule_home_request_reset(1.5)

class DraggableBall(QGraphicsEllipseItem):
    """Ball item that can be dragged and snaps to grid points on release."""
    def __init__(self, x, y, width, height, field_view):
        super().__init__(x, y, width, height)
        self.field_view = field_view
        self.is_dragging = False
        self.setAcceptHoverEvents(True)
        self.setFlag(self.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(self.GraphicsItemFlag.ItemSendsGeometryChanges, True)
    
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.is_dragging = True
            # anchor rect-based positioning so we can setRect during drag
            self.setPos(0, 0)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        # During drag, constrain X to grid columns and Z to [0, 1.25]m range
        if not self.is_dragging:
            return super().mouseMoveEvent(event)
        scene_p = event.scenePos()
        # convert to world coords
        wx, wz = self.field_view.scene_to_world(scene_p)
        gs = self.field_view.grid_spacing_m
        xmin = self.field_view.x_min_m
        xmax = self.field_view.x_max_m
        # snap x to nearest grid multiple
        n = round((wx - xmin) / gs)
        snapped_x_m = xmin + n * gs
        snapped_x_m = max(min(snapped_x_m, xmax), xmin)
        # Constrain Z to [0, 1.25]m range
        clamped_z_m = max(min(wz, 1.25), 0)
        # get scene coords for snapped x and clamped z
        snapped_scene_pos = self.field_view.world_to_scene(snapped_x_m, clamped_z_m)
        center_x = snapped_scene_pos.x()
        center_y = snapped_scene_pos.y()
        r = self.field_view.ball_r_px
        self.setRect(center_x - r, center_y - r, 2 * r, 2 * r)
        event.accept()
    
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self.is_dragging:
            self.is_dragging = False
            # Snap to nearest grid point after dragging
            ball_center = self.boundingRect().center()
            ball_scene_pos = self.mapToScene(ball_center)
            idx = self.field_view._nearest_grid_index(ball_scene_pos)
            if idx is not None:
                snap_scene = self.field_view.grid_points_scene[idx]
                snap_world = self.field_view.grid_points_world[idx]
                
                # Enforce ball height cap (1.25m max) and lower limit (0m min)
                if snap_world[1] > 1.25 or snap_world[1] < 0:
                    # Don't allow snapping above cap or below 0m — revert visually to last valid position
                    last = self.field_view.last_valid_ball_pos_scene
                    if last is not None:
                        self.setPos(0, 0)
                        self.setRect(
                            last.x() - self.field_view.ball_r_px,
                            last.y() - self.field_view.ball_r_px,
                            2 * self.field_view.ball_r_px,
                            2 * self.field_view.ball_r_px,
                        )
                        # update status to last valid world coords
                        last_world = self.field_view.scene_to_world(last)
                        self.field_view._update_status(*last_world)
                    return  # Don't snap if outside valid height range [0, 1.25]m
                
                # Reset item position and use setRect to position ball correctly
                self.setPos(0, 0)
                self.setRect(
                    snap_scene.x() - self.field_view.ball_r_px,
                    snap_scene.y() - self.field_view.ball_r_px,
                    2 * self.field_view.ball_r_px,
                    2 * self.field_view.ball_r_px
                )
                # Save as last valid position and update status
                self.field_view.last_valid_ball_pos_scene = snap_scene
                self.field_view._update_status(*snap_world)
        super().mouseReleaseEvent(event)


class DraggableDepthBall(QGraphicsEllipseItem):
    """Depth-view ball item that can be dragged and snaps to lower grid points."""
    def __init__(self, x, y, width, height, field_view):
        super().__init__(x, y, width, height)
        self.field_view = field_view
        self.is_dragging = False
        self.setAcceptHoverEvents(True)
        self.setFlag(self.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(self.GraphicsItemFlag.ItemSendsGeometryChanges, True)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.is_dragging = True
            self.setPos(0, 0)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if not self.is_dragging:
            return super().mouseMoveEvent(event)

        scene_p = event.scenePos()
        wx, wd = self.field_view.scene_to_world_depth(scene_p)
        gs = self.field_view.grid_spacing_m
        xmin = self.field_view.x_min_m
        xmax = self.field_view.x_max_m

        n = round((wx - xmin) / gs)
        snapped_x_m = xmin + n * gs
        snapped_x_m = max(min(snapped_x_m, xmax), xmin)

        clamped_depth_m = max(min(wd, self.field_view.depth_ball_max_y_m), self.field_view.depth_min_m)

        snapped_scene_pos = self.field_view.world_to_scene_depth(snapped_x_m, clamped_depth_m)
        center_x = snapped_scene_pos.x()
        center_y = snapped_scene_pos.y()
        r = self.field_view.ball_r_px
        self.setRect(center_x - r, center_y - r, 2 * r, 2 * r)
        event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self.is_dragging:
            self.is_dragging = False
            ball_center = self.boundingRect().center()
            ball_scene_pos = self.mapToScene(ball_center)
            idx = self.field_view._nearest_depth_grid_index(ball_scene_pos)
            if idx is not None:
                snap_scene = self.field_view.depth_grid_points_scene[idx]
                snap_world = self.field_view.depth_grid_points_world[idx]

                if snap_world[1] > self.field_view.depth_ball_max_y_m:
                    last = self.field_view.last_valid_depth_ball_pos_scene
                    if last is not None:
                        self.setPos(0, 0)
                        self.setRect(
                            last.x() - self.field_view.ball_r_px,
                            last.y() - self.field_view.ball_r_px,
                            2 * self.field_view.ball_r_px,
                            2 * self.field_view.ball_r_px,
                        )
                    return

                self.setPos(0, 0)
                self.setRect(
                    snap_scene.x() - self.field_view.ball_r_px,
                    snap_scene.y() - self.field_view.ball_r_px,
                    2 * self.field_view.ball_r_px,
                    2 * self.field_view.ball_r_px
                )
                self.field_view.last_valid_depth_ball_pos_scene = snap_scene
                self.field_view._update_depth_from_lower_grid(*snap_world)
        super().mouseReleaseEvent(event)


class ParabolaItem(QGraphicsPathItem):
    """Parabola with height adjustable via buttons."""
    def __init__(self, field_view, start_scene: QPointF, end_scene: QPointF):
        super().__init__()
        self.field_view = field_view
        self.start = start_scene
        self.end = end_scene
        self.setPen(QPen(Qt.GlobalColor.blue, 8))
        # Control point (peak) position
        y_offset = 30.0
        mid_x = (self.start.x() + self.end.x()) / 2
        peak_y = min(self.start.y(), self.end.y()) - 100 + y_offset
        self.ctrl = QPointF(mid_x, peak_y)
        self.update_path()

    def update_end(self, end_scene: QPointF):
        self.end = end_scene
        # Move control point horizontally to new midpoint while keeping y
        mid_x = (self.start.x() + self.end.x()) / 2
        self.ctrl.setX(mid_x)
        self.update_path()

    def update_path(self):
        # Build path using coordinates converted from scene -> this item's local coords
        path = QPainterPath()
        start_local = self.mapFromScene(self.start)
        end_local = self.mapFromScene(self.end)
        ctrl_local = self.mapFromScene(self.ctrl)
        path.moveTo(start_local)
        path.quadTo(ctrl_local, end_local)
        self.setPath(path)

    def increase_height(self):
        """Raise the parabola peak, capped at max height and concavity constraint."""
        # Convert ctrl point to world coords to check cap
        ctrl_world_x, ctrl_world_y = self.field_view.scene_to_world(self.ctrl)
        max_peak_y_m = self.field_view.parabola_max_peak_y_m
        
        # Only increase if below cap
        if ctrl_world_y < max_peak_y_m:
            self.ctrl.setY(self.ctrl.y() - 20)  # negative = up in scene coords
            # Enforce concavity: control point must be above the start-end line
            self._clamp_to_concave()
            self.update_path()
            # Update display with current ball position and new parabola height
            self.field_view._update_status(*self.field_view.ball_pos_world)

    def decrease_height(self):
        """Lower the parabola peak, enforcing concavity."""
        new_y = self.ctrl.y() + 20  # positive = down in scene coords
        self.ctrl.setY(new_y)
        # Enforce concavity: control point must be above the start-end line
        self._clamp_to_concave()
        self.update_path()
        # Update display with current ball position and new parabola height
        self.field_view._update_status(*self.field_view.ball_pos_world)

    def _clamp_to_concave(self):
        """Ensure control point stays above the line connecting start and end (concave arc)."""
        # Line from start to end: y = start.y + (end.y - start.y) * (x - start.x) / (end.x - start.x)
        # At ctrl.x, the line y value is:
        if abs(self.end.x() - self.start.x()) < 0.01:  # avoid division by zero
            line_y = (self.start.y() + self.end.y()) / 2
        else:
            t = (self.ctrl.x() - self.start.x()) / (self.end.x() - self.start.x())
            line_y = self.start.y() + t * (self.end.y() - self.start.y())
        # Clamp ctrl to be at or above the line (concave)
        if self.ctrl.y() > line_y:  # in scene coords, down is positive, so we want ctrl.y < line_y
            self.ctrl.setY(line_y)

class FieldView(QGraphicsView):
    """
    2D field view:
    - world coords (meters): x right, y up
    - scene coords (pixels): x right, y down
    """
    def __init__(self, status_label: QLabel, parent=None):
        super().__init__(parent) 
        self.status_label = status_label

        # ---- World configuration (edit these freely) ----
        self.grid_spacing_m = 0.25
        self.x_min_m, self.x_max_m = -6.5, 2.5
        self.z_min_m, self.z_max_m = -1.0, 4.0
        self.net_z_min = -1.0
        self.net_z_max = 0.0
        self.depth_min_m, self.depth_max_m = -3.0, 0.0
        self.depth_ball_max_y_m = -0.25
        self.lower_grid_gap_px = 140.0

        # Pixels per meter scale (zoom)
        self.ppm = 120.0  # 120 px = 1 meter

        # Snap threshold (pixels)
        self.snap_thresh_px = 30.0

        # Visual sizes (pixels)
        self.point_r_px = 2.5
        self.ball_r_px = 24.0

        # Scene setup
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(self.renderHints().Antialiasing, True)

        self.grid_points_scene = []   # list[QPointF] in scene coords
        self.grid_points_world = []   # list[(x_m, z_m)] matching indices
        self.depth_grid_points_scene = []  # list[QPointF] for lower depth grid
        self.depth_grid_points_world = []  # list[(x_m, y_m)] matching indices
        self.ball_pos_world = (0.0, 0.0)  # Current ball position in world coords (x_m, z_m)
        self.ball_y_m = 0.0
        self.depth_line = None
        self.last_valid_ball_pos_scene = None  # last valid scene position for the ball
        self.last_valid_depth_ball_pos_scene = None  # last valid scene position for lower ball
        
        # Parabola peak height cap (in world meters, adjustable for tuning)
        self.parabola_max_peak_y_m = 7.5
        # Minimum gap (meters) the parabola peak must be above the highest endpoint
        self.parabola_min_peak_gap_m = 0.1

        self._draw_everything()

        # Make view nicer
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)

    # ---------- Coordinate transforms ----------
    def world_to_scene(self, x_m: float, z_m: float) -> QPointF:
        """
        World (m): y up
        Scene (px): y down
        """
        x_px = x_m * self.ppm
        y_px = -z_m * self.ppm
        return QPointF(x_px, y_px)

    def scene_to_world(self, p: QPointF) -> tuple[float, float]:
        x_m = p.x() / self.ppm
        z_m = -p.y() / self.ppm
        return (x_m, z_m)

    def depth_origin_scene_y(self) -> float:
        top_bottom_scene = self.world_to_scene(0.0, self.z_min_m).y()
        return top_bottom_scene + self.lower_grid_gap_px

    def world_to_scene_depth(self, x_m: float, y_m: float) -> QPointF:
        x_px = x_m * self.ppm
        y_px = self.depth_origin_scene_y() + (-y_m * self.ppm)
        return QPointF(x_px, y_px)

    def scene_to_world_depth(self, p: QPointF) -> tuple[float, float]:
        x_m = p.x() / self.ppm
        y_m = -(p.y() - self.depth_origin_scene_y()) / self.ppm
        return (x_m, y_m)

    # ---------- Drawing ----------
    def _draw_everything(self):
        self.scene.clear()
        self.grid_points_scene.clear()
        self.grid_points_world.clear()
        self.depth_grid_points_scene.clear()
        self.depth_grid_points_world.clear()

        # Set scene rect to include full world bounds (plus margin)
        tl = self.world_to_scene(self.x_min_m, self.z_max_m)
        br = self.world_to_scene(self.x_max_m, self.z_min_m)
        depth_tl = self.world_to_scene_depth(self.x_min_m, self.depth_max_m)
        depth_br = self.world_to_scene_depth(self.x_max_m, self.depth_min_m)
        margin = 100
        self.scene.setSceneRect(
            min(tl.x(), br.x(), depth_tl.x(), depth_br.x()) - margin,
            min(tl.y(), br.y(), depth_tl.y(), depth_br.y()) - margin,
            abs(max(tl.x(), br.x(), depth_tl.x(), depth_br.x()) - min(tl.x(), br.x(), depth_tl.x(), depth_br.x())) + 2 * margin,
            abs(max(tl.y(), br.y(), depth_tl.y(), depth_br.y()) - min(tl.y(), br.y(), depth_tl.y(), depth_br.y())) + 2 * margin,
        )

        # Draw grid points
        point_pen = QPen(Qt.PenStyle.NoPen)
        point_brush = QBrush(Qt.GlobalColor.black)

        x = self.x_min_m
        while x <= self.x_max_m + 1e-9:
            z = self.z_min_m
            while z <= self.z_max_m + 1e-9:
                # Only add grid points for z >= 0
                if z >= 0:
                    sp = self.world_to_scene(x, z)
                    dot = QGraphicsEllipseItem(
                        sp.x() - self.point_r_px,
                        sp.y() - self.point_r_px,
                        2 * self.point_r_px,
                        2 * self.point_r_px
                    )
                    dot.setPen(point_pen)
                    dot.setBrush(point_brush)
                    dot.setZValue(0)
                    self.scene.addItem(dot)

                    self.grid_points_scene.append(sp)
                    self.grid_points_world.append((x, z))
                z += self.grid_spacing_m
            x += self.grid_spacing_m

        # Draw lower depth grid points (x vs y)
        x = self.x_min_m
        while x <= self.x_max_m + 1e-9:
            y_m = self.depth_min_m
            while y_m <= self.depth_max_m + 1e-9:
                sp = self.world_to_scene_depth(x, y_m)
                dot = QGraphicsEllipseItem(
                    sp.x() - self.point_r_px,
                    sp.y() - self.point_r_px,
                    2 * self.point_r_px,
                    2 * self.point_r_px
                )
                dot.setPen(point_pen)
                dot.setBrush(point_brush)
                dot.setZValue(0)
                self.scene.addItem(dot)

                self.depth_grid_points_scene.append(sp)
                self.depth_grid_points_world.append((x, y_m))
                y_m += self.grid_spacing_m
            x += self.grid_spacing_m

        # Label lower grid
        lower_label = QGraphicsTextItem("NET")
        lower_label.setDefaultTextColor(Qt.GlobalColor.white)
        lower_label.setZValue(2)
        lower_label_pos = self.world_to_scene_depth(self.x_min_m, self.depth_max_m)
        lower_label.setPos(lower_label_pos.x(), lower_label_pos.y() - 28)
        self.scene.addItem(lower_label)

        # Draw depth-grid net reference line at y = 0 across x
        depth_net_pen = QPen(Qt.GlobalColor.white)
        depth_net_pen.setWidth(8)
        depth_left = self.world_to_scene_depth(self.x_min_m, 0.0)
        depth_right = self.world_to_scene_depth(self.x_max_m, 0.0)
        depth_net_line = QGraphicsLineItem(
            depth_left.x(),
            depth_left.y(),
            depth_right.x(),
            depth_right.y()
        )
        depth_net_line.setPen(depth_net_pen)
        depth_net_line.setZValue(0.6)
        self.scene.addItem(depth_net_line)

        # Draw net (simple line at y = net_y_m)
        net_pen = QPen(Qt.GlobalColor.gray)
        net_pen.setWidth(4)
        left = self.world_to_scene(self.x_min_m, self.net_z_min)
        right = self.world_to_scene(self.x_max_m, self.net_z_max)
        print(left, right)
        net = QGraphicsRectItem(left.x(), left.y(), right.x()-left.x(), right.y()-left.y())
        net.setPen(net_pen)        
        net_brush = QBrush(Qt.GlobalColor.white, Qt.BrushStyle.CrossPattern)
        net.setBrush(net_brush)        
        net.setZValue(0.5)
        self.scene.addItem(net)

        # Add a small text item under the net that will show the ball coordinates
        self.net_text = QGraphicsTextItem()
        self.net_text.setDefaultTextColor(Qt.GlobalColor.white)
    
        self.net_text.setZValue(3)
        center_x = (left.x() + right.x()) / 2
        bottom_y = max(left.y(), right.y())
        # Position a bit below the net
        self.net_text.setPos(center_x - 120, bottom_y + 8)
        self.scene.addItem(self.net_text)

        # Create ball at an initial snapped point (0, 1m) if possible
        init_world = (-1.0, 1.0)
        init_scene = self.world_to_scene(*init_world)
        idx = self._nearest_grid_index(init_scene)
        if idx is not None:
            init_scene = self.grid_points_scene[idx]
            init_world = self.grid_points_world[idx]

        ball_pen = QPen(Qt.GlobalColor.darkYellow)
        ball_pen.setWidth(2)
        ball_brush = QBrush(Qt.GlobalColor.yellow)

        self.ball = DraggableBall(
            init_scene.x() - self.ball_r_px,
            init_scene.y() - self.ball_r_px,
            2 * self.ball_r_px,
            2 * self.ball_r_px,
            self
        )
        self.ball.setPen(ball_pen)
        self.ball.setBrush(ball_brush)
        self.ball.setZValue(2)
        self.scene.addItem(self.ball)
        
        # Initialize last valid position with the ball's starting position
        self.last_valid_ball_pos_scene = init_scene

        # Create lower-grid depth ball at same x and clamped depth limit
        init_depth_world = (init_world[0], self.depth_ball_max_y_m)
        init_depth_scene = self.world_to_scene_depth(*init_depth_world)
        depth_idx = self._nearest_depth_grid_index(init_depth_scene)
        if depth_idx is not None:
            init_depth_scene = self.depth_grid_points_scene[depth_idx]
            init_depth_world = self.depth_grid_points_world[depth_idx]

        self.depth_ball = DraggableDepthBall(
            init_depth_scene.x() - self.ball_r_px,
            init_depth_scene.y() - self.ball_r_px,
            2 * self.ball_r_px,
            2 * self.ball_r_px,
            self
        )
        self.depth_ball.setPen(ball_pen)
        self.depth_ball.setBrush(ball_brush)
        self.depth_ball.setZValue(2)
        self.scene.addItem(self.depth_ball)
        self.last_valid_depth_ball_pos_scene = init_depth_scene
        self.ball_y_m = init_depth_world[1]

        # Depth-view line from origin (x=0, y=0) to lower-grid ball
        depth_line_pen = QPen(Qt.GlobalColor.blue)
        depth_line_pen.setWidth(8)
        depth_line_start = self.world_to_scene_depth(0.0, 0.0)
        depth_line_end = self.world_to_scene_depth(init_world[0], self.ball_y_m)
        self.depth_line = QGraphicsLineItem(
            depth_line_start.x(),
            depth_line_start.y(),
            depth_line_end.x(),
            depth_line_end.y()
        )
        self.depth_line.setPen(depth_line_pen)
        self.depth_line.setZValue(0.8)
        self.scene.addItem(self.depth_line)

        # Create parabola from (0,0) world to the initial ball location
        start_scene = self.world_to_scene(0.0, 0.0)
        end_scene = self.world_to_scene(*init_world)
        self.parabola = ParabolaItem(self, start_scene, end_scene)
        self.parabola.setZValue(0.8)
        self.scene.addItem(self.parabola)

        self._update_status(*init_world)



    # ---------- Interaction ----------
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            scene_pos = self.mapToScene(event.position().toPoint())
            # If an item was clicked (ball, handle, etc.), let the item handle the event
            item = self.scene.itemAt(scene_pos, self.transform())
            if item is not None:
                super().mousePressEvent(event)
                return

            # No item clicked: pass to base to allow panning
            super().mousePressEvent(event)
            return

        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)

    def _move_ball_to_scene(self, p: QPointF):
        # Add vertical offset to make ball appear lower
        y_offset = 500.0
        self.ball.setRect(
            p.x() - self.ball_r_px,
            p.y() + y_offset - self.ball_r_px,
            2 * self.ball_r_px,
            2 * self.ball_r_px
        )

    def _nearest_grid_index(self, scene_pos: QPointF):
        if not self.grid_points_scene:
            return None

        best_i = None
        best_d2 = float("inf")
        for i, gp in enumerate(self.grid_points_scene):
            dx = gp.x() - scene_pos.x()
            dy = gp.y() - scene_pos.y()
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        if best_i is None:
            return None

        if math.sqrt(best_d2) <= self.snap_thresh_px:
            return best_i
        return None

    def _nearest_depth_grid_index(self, scene_pos: QPointF):
        if not self.depth_grid_points_scene:
            return None

        best_i = None
        best_d2 = float("inf")
        for i, gp in enumerate(self.depth_grid_points_scene):
            dx = gp.x() - scene_pos.x()
            dy = gp.y() - scene_pos.y()
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_i = i

        if best_i is None:
            return None

        if math.sqrt(best_d2) <= self.snap_thresh_px:
            return best_i
        return None

    def _sync_depth_ball_from_state(self):
        clamped_y_m = max(min(self.ball_y_m, self.depth_ball_max_y_m), self.depth_min_m)
        target_scene = self.world_to_scene_depth(self.ball_pos_world[0], clamped_y_m)
        idx = self._nearest_depth_grid_index(target_scene)
        if idx is None:
            return
        snap_scene = self.depth_grid_points_scene[idx]
        snap_world = self.depth_grid_points_world[idx]
        self.depth_ball.setPos(0, 0)
        self.depth_ball.setRect(
            snap_scene.x() - self.ball_r_px,
            snap_scene.y() - self.ball_r_px,
            2 * self.ball_r_px,
            2 * self.ball_r_px
        )
        self.last_valid_depth_ball_pos_scene = snap_scene
        self.ball_y_m = min(snap_world[1], self.depth_ball_max_y_m)

    def _sync_top_ball_x_from_state(self):
        x_m, z_m = self.ball_pos_world
        target_scene = self.world_to_scene(x_m, z_m)
        idx = self._nearest_grid_index(target_scene)
        if idx is None:
            return
        snap_scene = self.grid_points_scene[idx]
        snap_world = self.grid_points_world[idx]
        self.ball.setPos(0, 0)
        self.ball.setRect(
            snap_scene.x() - self.ball_r_px,
            snap_scene.y() - self.ball_r_px,
            2 * self.ball_r_px,
            2 * self.ball_r_px
        )
        self.last_valid_ball_pos_scene = snap_scene
        self.ball_pos_world = (snap_world[0], snap_world[1])

    def _update_depth_from_lower_grid(self, x_m: float, y_m: float):
        old_z = self.ball_pos_world[1]
        self.ball_pos_world = (x_m, old_z)
        self.ball_y_m = max(min(y_m, self.depth_ball_max_y_m), self.depth_min_m)
        self._sync_top_ball_x_from_state()
        self._update_status(*self.ball_pos_world)

    def _update_depth_line(self):
        if not hasattr(self, 'depth_line') or self.depth_line is None:
            return
        start = self.world_to_scene_depth(0.0, 0.0)
        end = self.world_to_scene_depth(self.ball_pos_world[0], self.ball_y_m)
        self.depth_line.setLine(start.x(), start.y(), end.x(), end.y())

    def _update_status(self, x_m: float, z_m: float):
        global setter_state
        self.ball_pos_world = (x_m, z_m)

        g = 9.81

        # Keep visual state in sync before using values for calculations
        if hasattr(self, 'parabola') and self.parabola is not None:
            end_scene = self.world_to_scene(x_m, z_m)
            self.parabola.update_end(end_scene)
        if hasattr(self, 'depth_ball') and self.depth_ball is not None:
            self._sync_depth_ball_from_state()

        # Get actual parabola peak height in world coords (z_peak)
        parabola_height_m = 0.0
        if hasattr(self, 'parabola') and self.parabola is not None:
            start_world_x, start_world_y = self.scene_to_world(self.parabola.start)
            ctrl_world_x, ctrl_world_y = self.scene_to_world(self.parabola.ctrl)
            end_world_x, end_world_y = self.scene_to_world(self.parabola.end)

            z0 = start_world_y   # launch height from parabola start
            z1 = ctrl_world_y
            z2 = end_world_y     # should correspond to target z (close to z_m)

            denom = (z0 - 2 * z1 + z2)
            if abs(denom) < 1e-9:
                parabola_height_m = max(z0, z2)
            else:
                t_peak = (z0 - z1) / denom
                t_peak = max(0.0, min(1.0, t_peak))
                one_minus_t = 1.0 - t_peak
                z_peak = (
                    (one_minus_t * one_minus_t * z0)
                    + (2 * one_minus_t * t_peak * z1)
                    + (t_peak * t_peak * z2)
                )
                parabola_height_m = max(z0, z2, z_peak)
        else:
            # Fallback if no parabola exists; choose something valid above both start and target
            z0 = 0.0
            parabola_height_m = max(z0, z_m) + 0.25

        # Define launch and target heights in world coordinates
        # IMPORTANT:
        # If your launcher is not at z=0, replace z_launch with the true launcher height.
        z_launch = z0 if 'z0' in locals() else 0.0
        z_target = z_m
        dz = z_target - z_launch

        # Horizontal displacement magnitude in x-y plane (3D horizontal range)
        dist = math.sqrt(x_m * x_m + self.ball_y_m * self.ball_y_m)

        # Ensure peak is physically valid (must be at/above both launch and target heights)
        z_peak = max(parabola_height_m, z_launch, z_target)

        # --- Peak-height-based solve ---
        # Vertical launch speed required to reach z_peak
        vz_sq = 2.0 * g * (z_peak - z_launch)
        vz_sq = max(0.0, vz_sq)
        V_z = math.sqrt(vz_sq)

        # Time to hit z_target on descending branch:
        # z_target = z_launch + V_z*T - 0.5*g*T^2
        # => T = (V_z + sqrt(V_z^2 - 2*g*dz)) / g
        disc = V_z * V_z - 2.0 * g * dz
        if disc < 0.0:
            # Shouldn't happen if z_peak >= max(z_launch, z_target), but clamp for robustness
            disc = 0.0

        t = (V_z + math.sqrt(disc)) / g

        # Guard against divide-by-zero (degenerate case)
        if t < 1e-6:
            t = 1e-6

        # Horizontal launch speed magnitude
        V_h = dist / t

        # If you need actual x/y components (for yaw + actuator decomposition)
        V_x = (x_m / t) if abs(t) > 1e-9 else 0.0
        V_y = (self.ball_y_m / t) if abs(t) > 1e-9 else 0.0

        # Launch angles
        th = math.atan2(V_z, V_h)              # elevation / pitch
        yaw = abs(math.atan2(self.ball_y_m, x_m))   # azimuth / yaw

        # Exit speed magnitude
        V_0 = math.sqrt(V_h * V_h + V_z * V_z)

        # Optional: actual apex check (should match z_peak closely)
        z_peak_actual = z_launch + (V_z * V_z) / (2.0 * g)

        # Optional: keep your existing status/debug fields updated
        # (rename/add based on what your UI expects)
        self.last_launch_solution = {
            "x_m": x_m,
            "y_m": self.ball_y_m,
            "z_target_m": z_target,
            "z_launch_m": z_launch,
            "z_peak_m": z_peak,
            "z_peak_actual_m": z_peak_actual,
            "flight_time_s": t,
            "V_x_mps": V_x,
            "V_y_mps": V_y,
            "V_h_mps": V_h,
            "V_z_mps": V_z,
            "V_0_mps": V_0,
            "theta_rad": th,
            "theta_deg": math.degrees(th),
            "yaw_rad": yaw,
            "yaw_deg": math.degrees(yaw),
        }
        
        # Update global state
        setter_state['ball_x'] = x_m
        setter_state['ball_z'] = z_m
        setter_state['ball_y'] = self.ball_y_m
        setter_state['parabola_peak'] = parabola_height_m
        setter_state['initial_velocity'] = V_0
        setter_state['launch_angle'] = math.degrees(th)
        setter_state['yaw_angle'] = math.degrees(yaw)
        
        
        self.status_label.setText(f"Target (m): x = {x_m:.2f}, z = {z_m:.2f}, y = {self.ball_y_m:.2f}, yaw = {math.degrees(yaw):.1f}°")
        # Update scene text under the net if present
        if hasattr(self, 'net_text') and self.net_text is not None:
            self.net_text.setPlainText(f"x = {x_m:.2f} m, z = {z_m:.2f} m, y = {self.ball_y_m:.2f} m, peak = {parabola_height_m:.2f} m, velocity = {V_0:.2f} m/s, angle = {math.degrees(th):.1f}°, yaw = {math.degrees(yaw):.1f}°, velocity_x = {V_x:.2f} m/s, velocity_z = {V_z:.2f} m/s")
            # Reposition in case scene or net changed
            left = self.world_to_scene(self.x_min_m, self.net_z_min)
            right = self.world_to_scene(self.x_max_m, self.net_z_max)
            center_x = (left.x() + right.x()) / 2
            bottom_y = max(left.y(), right.y())
            self.net_text.setPos(center_x - 120, bottom_y + 8)
        self._update_depth_line()

    def get_ball_position(self) -> tuple[float, float]:
        """Returns ball position as (x_m, z_m) in world coordinates."""
        return self.ball_pos_world


class MainWindow(QWidget):
    def __init__(self, ros_node: OnsetbotGuiRosNode | None = None):
        super().__init__()
        self.setWindowTitle("Volleyball Setter GUI - Grid Picker")
        self.ros_node = ros_node

        # Timer for hold-to-repeat button functionality
        self.hold_timer = QTimer()
        self.hold_timer.timeout.connect(self.on_hold_timer)
        self.held_button = None  # track which button is held

        # Timer to update launch button readiness
        self.launch_ready_timer = QTimer()
        self.launch_ready_timer.timeout.connect(self.update_launch_button_state)

        main_layout = QHBoxLayout(self)

        # Left side: field view
        left_layout = QVBoxLayout()
        self.status = QLabel("Target (m): x = --, z = --, y = --")
        self.status.setAlignment(Qt.AlignmentFlag.AlignLeft)

        self.view = FieldView(self.status)

        left_layout.addWidget(self.view)
        left_layout.addWidget(self.status)

        # Right side: parabola height buttons
        right_layout = QVBoxLayout()
        right_layout.addStretch()
        
        btn_up = QPushButton("↑ Higher")
        btn_down = QPushButton("↓ Lower")
        self.btn_launch = QPushButton("Launch")
        btn_home_robot = QPushButton("Home Robot")
        
        # Connect pressed/released for hold-to-repeat
        btn_up.pressed.connect(lambda: self.on_button_pressed("up"))
        btn_up.released.connect(self.on_button_released)
        btn_down.pressed.connect(lambda: self.on_button_pressed("down"))
        btn_down.released.connect(self.on_button_released)
        
        self.btn_launch.clicked.connect(self.on_launch_clicked)
        btn_home_robot.clicked.connect(self.home_robot_command)

        right_layout.addWidget(btn_up)
        right_layout.addWidget(btn_down)
        right_layout.addWidget(self.btn_launch)
        right_layout.addWidget(btn_home_robot)
        right_layout.addStretch()

        main_layout.addLayout(left_layout, 1)
        main_layout.addLayout(right_layout)

        self.setLayout(main_layout)
        self.resize(1400, 900)

        self.btn_launch.setEnabled(False)
        self.launch_ready_timer.start(100)

    def increase_parabola_height(self):
        """Raise the parabola."""
        self.view.parabola.increase_height()

    def decrease_parabola_height(self):
        """Lower the parabola."""
        self.view.parabola.decrease_height()

    def on_button_pressed(self, direction):
        """Start hold-to-repeat timer."""
        self.held_button = direction
        self.hold_timer.start(50)  # 50ms interval

    def on_button_released(self):
        """Stop hold-to-repeat timer."""
        self.hold_timer.stop()
        self.held_button = None

    def on_hold_timer(self):
        """Called by timer to continuously adjust."""
        if self.held_button == "up":
            self.increase_parabola_height()
        elif self.held_button == "down":
            self.decrease_parabola_height()

    def update_launch_button_state(self):
        if self.ros_node is None:
            self.btn_launch.setEnabled(False)
            return
        self.btn_launch.setEnabled(self.ros_node.can_launch())

    def on_launch_clicked(self):
        """Publish launch info only when the Launch button is clicked."""
        if self.ros_node is None:
            return
        if not self.ros_node.can_launch():
            return
        solution = getattr(self.view, "last_launch_solution", None)
        if not solution:
            return
        self.ros_node.publish_odrive_command(
            solution["V_0_mps"],
            solution["yaw_rad"],
            solution["theta_rad"],
        )

    def home_robot_command(self):
        """Placeholder for robot homing command."""
        if self.ros_node is None:
            return
        self.ros_node.request_home_onset()

def main():
    rclpy.init(args=None)

    ros_node = OnsetbotGuiRosNode()

    app = QApplication(sys.argv)

    def _handle_sigint(_sig, _frame):
        app.quit()

    signal.signal(signal.SIGINT, _handle_sigint)

    w = MainWindow(ros_node=ros_node)
    w.show()

    # Periodically pump ROS callbacks without blocking the Qt event loop
    ros_spin_timer = QTimer()
    ros_spin_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    ros_spin_timer.start(10)  # 10 ms ~= 100 Hz spin servicing

    try:
        exit_code = app.exec()
    finally:
        ros_spin_timer.stop()
        ros_node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)

if __name__ == "__main__":
    main()
