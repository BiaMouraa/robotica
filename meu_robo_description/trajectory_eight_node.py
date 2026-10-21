import math
from sensor_msgs.msg import Imu, NavSatFix

# Inicializa variáveis de posição e orientação globais
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.lat_inicial = None
        self.lon_inicial = None

        # Cria os subscribers para a IMU e o GPS
        self.subscription_imu = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.subscription_gps = self.create_subscription(NavSatFix, '/gps/data', self.gps_callback, 10)

def imu_callback(self, msg):
        # Converte a orientação em Quaternions da IMU para o ângulo Yaw (Z)
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def gps_callback(self, msg):
        # Define o ponto zero (origem) na primeira leitura do GPS
        if self.lat_inicial is None:
            self.lat_inicial = msg.latitude
            self.lon_inicial = msg.longitude

        # Converte as coordenadas globais (Lat/Lon) para posições locais Cartesianas (X, Y)
        raio_terra = 6371000.0
        delta_lat = math.radians(msg.latitude - self.lat_inicial)
        delta_lon = math.radians(msg.longitude - self.lon_inicial)

        # Atualiza a posição X e Y do robô baseada no GPS
        self.current_x = delta_lon * raio_terra * math.cos(math.radians(self.lat_inicial))
        self.current_y = delta_lat * raio_terra
