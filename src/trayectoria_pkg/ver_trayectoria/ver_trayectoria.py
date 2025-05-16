import rclpy
from rclpy.node import Node
import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from scipy.spatial import distance

from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped


class VerTrayectoria(Node):

    def __init__(self):
        super().__init__('ver_trayectoria')

        self.is_real = False
        self.map_name = 'pruebas2'

        odom_topic = '/pf/viz/inferred_pose' if self.is_real else '/ego_racecar/odom'
        visualization_topic = '/trayectoria_recorrida'

        # Cargar waypoints y velocidades de referencia
        map_path = os.path.abspath(os.path.join('src', 'csv_data'))
        csv_data = np.loadtxt(map_path + '/' + self.map_name + '.csv', delimiter=',', skiprows=0)
        self.waypoints = csv_data[:, 1:3]  # Coordenadas (X, Y)
        self.reference_speeds = csv_data[:, 5]  # Velocidades de referencia

        # Suscripción a la pose
        self.sub_pose = self.create_subscription(Odometry, odom_topic, self.pose_callback, 1)

        # Publicador de trayectorias
        self.marker_pub = self.create_publisher(MarkerArray, visualization_topic, 10)

        # Almacenar trayectoria y velocidades
        self.trajectory_points = []  # [(x, y, velocidad_real, velocidad_referencia, tiempo)]

        # Posición y tiempo de inicio
        self.start_pos = None
        self.start_time = None
        self.has_started = False

        # Imagen del mapa para visualización
        self.map_image_path = '/home/jaycee/sim_ws/src/f1tenth_gym_ros/maps/Oschersleben_map.png'

    def pose_callback(self, pose_msg):
        # Obtener posición actual
        self.currX = pose_msg.pose.pose.position.x
        self.currY = pose_msg.pose.pose.position.y
        self.currPos = np.array([self.currX, self.currY]).reshape((1, 2))

        # Obtener velocidad actual
        self.curr_speed = np.sqrt(
            pose_msg.twist.twist.linear.x ** 2 + pose_msg.twist.twist.linear.y ** 2
        )

        # Obtener el tiempo actual en segundos
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Inicializar tiempo de inicio
        if self.start_time is None:
            self.start_time = current_time

        # Calcular tiempo relativo desde el inicio
        relative_time = current_time - self.start_time

        # Buscar el punto de referencia más cercano
        nearest_idx = np.argmin(distance.cdist(self.currPos, self.waypoints))
        reference_speed_nearest = self.reference_speeds[nearest_idx]

        # Guardar el dato en la trayectoria
        self.trajectory_points.append((self.currX, self.currY, self.curr_speed, reference_speed_nearest, relative_time))

        # Verificar inicio de movimiento
        if self.start_pos is None:
            self.start_pos = self.currPos.flatten()

        if not self.has_started and distance.euclidean(self.start_pos, self.currPos.flatten()) > 0.5:
            self.has_started = True
            print("El coche ha comenzado a moverse.")

        # Verificar si el coche ha completado la vuelta
        if self.has_started and distance.euclidean(self.start_pos, self.currPos.flatten()) < 0.2:
            lap_time_sec = relative_time
            print(f"Tiempo de vuelta: {lap_time_sec:.2f} segundos")

            # Calcular discrepancias
            trajectory_discrepancy = self.calculate_discrepancy()
            speed_discrepancy = self.calculate_speed_discrepancy()

            print(f"Discrepancia en trayectoria: {trajectory_discrepancy:.2f}%")
            print(f"Discrepancia en velocidad: {speed_discrepancy:.2f}%")

            # Graficar los datos recopilados
            self.plot_trajectory()
            rclpy.shutdown()

    def calculate_discrepancy(self):
        # Calcular la discrepancia de la trayectoria como error porcentual
        trajectory_points_np = np.array([(x, y) for x, y, _, _, _ in self.trajectory_points])
        distances = np.min(distance.cdist(trajectory_points_np, self.waypoints), axis=1)
        discrepancy = (np.mean(distances) / np.mean(np.linalg.norm(self.waypoints, axis=1))) * 100
        return discrepancy

    def calculate_speed_discrepancy(self):
        # Calcular el error porcentual entre velocidades medias
        real_speeds = np.array([point[2] for point in self.trajectory_points])
        reference_speeds_nearest = np.array([point[3] for point in self.trajectory_points])

        mean_real_speed = np.mean(real_speeds)
        mean_reference_speed = np.mean(reference_speeds_nearest)

        speed_error_percent = abs(mean_real_speed - mean_reference_speed) / mean_reference_speed * 100
        return speed_error_percent

    def plot_trajectory(self):
        # Configurar estilo académico
        plt.rcParams["font.family"] = "Times New Roman"
        plt.rcParams["font.size"] = 14
        plt.rcParams["axes.labelweight"] = "bold"
        plt.rcParams["axes.titlesize"] = 16
        plt.rcParams["axes.titleweight"] = "bold"

        # Extraer datos de la trayectoria
        real_times = [point[4] for point in self.trajectory_points]
        real_speeds = [point[2] for point in self.trajectory_points]
        reference_speeds_nearest = [point[3] for point in self.trajectory_points]
        x_coords = [point[0] for point in self.trajectory_points]
        y_coords = [point[1] for point in self.trajectory_points]

        # Crear la figura con dos gráficos
        fig, axs = plt.subplots(1, 2, figsize=(14, 6))

         # Extraer waypoints y velocidades de referencia
        waypoint_x_coords = self.waypoints[:, 0]
        waypoint_y_coords = self.waypoints[:, 1]

        # 🔹 Gráfico de Trayectorias 🔹
        map_img = mpimg.imread(self.map_image_path)
        map_resolution = 0.04295
        map_origin = [-55.07650228661655, -33.57884064395765]
        map_width = 1600
        map_height = 1600
        map_extent = [
            map_origin[0],
            map_origin[0] + map_width * map_resolution,
            map_origin[1],
            map_origin[1] + map_height * map_resolution
        ]

        axs[0].imshow(map_img, cmap='gray', extent=map_extent)
        axs[0].plot(x_coords, y_coords, marker='o', color='red', markersize=0.5, linestyle='None', label='Trayectoria Real')
        axs[0].plot(waypoint_x_coords, waypoint_y_coords, marker='o', color='blue', markersize=0.5, linestyle='None', label='Referencia')
        axs[0].set_title('Trayectoria Seguida')
        axs[0].set_xlabel('Coordenada X (m)')
        axs[0].set_ylabel('Coordenada Y (m)')
        axs[0].legend()
        axs[0].grid(True)

        # 🔹 Gráfico de Velocidades en Tiempo Real 🔹
        axs[1].plot(real_times, real_speeds, color='red', linewidth=2, label='Velocidad real')
        axs[1].plot(real_times, reference_speeds_nearest, '--', color='blue', linewidth=2, label='Velocidad de referencia más cercana')
        axs[1].set_title('Velocidad Real vs. Velocidad de Referencia')
        axs[1].set_xlabel('Tiempo de vuelta (s)')
        axs[1].set_ylabel('Velocidad (m/s)')
        axs[1].legend()
        axs[1].grid(True, linestyle="--", alpha=0.7)

        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    print("Ver trayectoria iniciado")
    node = VerTrayectoria()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
