#!/usr/bin/env python3
"""
Test avanzato per Impedance Controller
Esegue movimenti complessi VISIBILI in spazio limitato
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
import numpy as np

class ImpedanceComplexTest(Node):
    def __init__(self):
        super().__init__('impedance_complex_test')
        
        # Publisher per target pose
        self.pub = self.create_publisher(PoseStamped, '/desired_pose', 10)
        
        # Subscriber per posizione corrente
        self.sub = self.create_subscription(
            PoseStamped, '/current_ee_pose',
            self.pose_callback, 10
        )
        
        # Stato
        self.current_pose = None
        self.pose_updated = False
        self.initial_position = None
        
        # Configurazione - RIDOTTO per movimenti più lenti e visibili
        self.control_rate = 10.0  # Hz - ridotto da 20 a 10 (più lento)
        self.dt = 1.0 / self.control_rate
        
        self.get_logger().info('='*70)
        self.get_logger().info('IMPEDANCE CONTROL - ADVANCED TEST (VISIBLE MOVEMENTS)')
        self.get_logger().info('='*70)
        self.get_logger().info('Attendo posizione corrente...')
    
    def pose_callback(self, msg):
        """Callback per aggiornare posizione corrente"""
        self.current_pose = msg
        self.pose_updated = True
    
    def wait_for_pose(self, timeout=5.0):
        """Aspetta che la posizione sia disponibile"""
        start = time.time()
        while not self.pose_updated and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not self.pose_updated:
            self.get_logger().error('❌ Timeout: posizione non ricevuta!')
            return False
        
        self.pose_updated = False
        return True
    
    def send_absolute_target(self, position_xyz):
        """Invia target assoluto mantenendo orientamento corrente"""
        if self.current_pose is None:
            return False
        
        target = PoseStamped()
        target.header.frame_id = 'world'
        target.header.stamp = self.get_clock().now().to_msg()
        
        target.pose.position.x = position_xyz[0]
        target.pose.position.y = position_xyz[1]
        target.pose.position.z = position_xyz[2]
        
        # Mantieni orientamento
        target.pose.orientation = self.current_pose.pose.orientation
        
        self.pub.publish(target)
        return True
    
    def execute_trajectory(self, name, waypoints, duration_per_segment, show_progress=True):
        """
        Esegue traiettoria definita da waypoints
        waypoints: lista di np.array([x, y, z]) relativi a initial_position
        duration_per_segment: secondi per ogni segmento
        show_progress: mostra posizione corrente durante movimento
        """
        self.get_logger().info('')
        self.get_logger().info(f'🔷 {name}')
        self.get_logger().info('-'*70)
        self.get_logger().info(f'   Waypoints: {len(waypoints)}')
        self.get_logger().info(f'   Durata totale: {len(waypoints)*duration_per_segment:.1f}s')
        self.get_logger().info(f'   Velocità: {self.control_rate} Hz')
        
        start_time = time.time()
        
        for i, waypoint in enumerate(waypoints):
            # Calcola posizione assoluta
            target_pos = self.initial_position + waypoint
            
            # Interpolazione lineare per movimento fluido
            steps = int(duration_per_segment * self.control_rate)
            
            if i == 0:
                # Primo waypoint: parti da posizione corrente
                start_pos = np.array([
                    self.current_pose.pose.position.x,
                    self.current_pose.pose.position.y,
                    self.current_pose.pose.position.z
                ])
            else:
                # Waypoint successivo: parti dal precedente
                start_pos = self.initial_position + waypoints[i-1]
            
            # Log waypoint
            if show_progress:
                self.get_logger().info(
                    f'   → Waypoint {i+1}/{len(waypoints)}: '
                    f'[{waypoint[0]*1000:+.1f}, {waypoint[1]*1000:+.1f}, {waypoint[2]*1000:+.1f}] mm'
                )
            
            # Interpola tra start_pos e target_pos
            for step in range(steps):
                alpha = (step + 1) / steps  # 0 a 1
                interpolated = start_pos + alpha * (target_pos - start_pos)
                
                self.send_absolute_target(interpolated)
                
                # Spin per ricevere feedback
                rclpy.spin_once(self, timeout_sec=0.001)
                
                # Feedback posizione corrente ogni 10 step
                if show_progress and step % 10 == 0 and self.current_pose:
                    curr = np.array([
                        self.current_pose.pose.position.x,
                        self.current_pose.pose.position.y,
                        self.current_pose.pose.position.z
                    ])
                    error = np.linalg.norm(curr - interpolated)
                    # Log compatto sulla stessa riga
                    print(f'\r      Step {step}/{steps}, Errore: {error*1000:.1f}mm', end='', flush=True)
                
                # Rate control
                time.sleep(self.dt)
            
            if show_progress:
                print()  # Newline dopo progress bar
        
        elapsed = time.time() - start_time
        
        # Verifica posizione finale
        if self.wait_for_pose(timeout=1.0):
            final_pos = np.array([
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z
            ])
            displacement = final_pos - self.initial_position
            error = np.linalg.norm(displacement - waypoints[-1])
            
            self.get_logger().info(
                f'   ✅ Completato in {elapsed:.1f}s (errore finale: {error*1000:.1f}mm)'
            )
            self.get_logger().info(
                f'      Posizione finale: [{final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f}]'
            )
    
    def generate_circle_xy(self, radius=0.03, num_points=16):
        """Cerchio sul piano XY - RADDOPPIATO (raggio 3cm)"""
        angles = np.linspace(0, 2*np.pi, num_points)
        waypoints = []
        for angle in angles:
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            waypoints.append(np.array([x, y, 0.0]))
        return waypoints
    
    def generate_circle_xz(self, radius=0.03, num_points=16):
        """Cerchio sul piano XZ - RADDOPPIATO (raggio 3cm)"""
        angles = np.linspace(0, 2*np.pi, num_points)
        waypoints = []
        for angle in angles:
            x = radius * np.cos(angle)
            z = radius * np.sin(angle)
            waypoints.append(np.array([x, 0.0, z]))
        return waypoints
    
    def generate_spiral_3d(self, radius=0.04, height=0.05, num_turns=1.5, num_points=24):
        """Spirale 3D - AUMENTATO (raggio 4cm, altezza 5cm)"""
        t = np.linspace(0, num_turns * 2 * np.pi, num_points)
        waypoints = []
        for i, angle in enumerate(t):
            # Raggio decresce verso il centro
            r = radius * (1 - i / len(t))
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            z = height * i / len(t)
            waypoints.append(np.array([x, y, z]))
        
        # Ritorno al centro lentamente
        waypoints.append(np.array([0.0, 0.0, height]))
        waypoints.append(np.array([0.0, 0.0, 0.0]))
        return waypoints
    
    def generate_star_pattern(self, radius=0.04, num_rays=6):
        """
        Pattern a stella - AUMENTATO (raggio 4cm, 6 raggi)
        """
        angles = np.linspace(0, 2*np.pi, num_rays, endpoint=False)
        waypoints = []
        
        for angle in angles:
            # Vai verso esterno
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            waypoints.append(np.array([x, y, 0.0]))
            
            # Torna al centro
            waypoints.append(np.array([0.0, 0.0, 0.0]))
        
        return waypoints
    
    def generate_figure_eight(self, size=0.035, num_points=32):
        """Figura a otto (lemniscata) - AUMENTATO (3.5cm)"""
        t = np.linspace(0, 2*np.pi, num_points)
        waypoints = []
        for angle in t:
            # Parametric equation for lemniscate
            scale = size / (1 + np.sin(angle)**2)
            x = scale * np.cos(angle)
            y = scale * np.sin(angle) * np.cos(angle)
            waypoints.append(np.array([x, y, 0.0]))
        return waypoints
    
    def generate_square(self, side=0.05, num_points_per_side=8):
        """Quadrato sul piano XY (lato 5cm)"""
        waypoints = []
        
        # 4 angoli del quadrato
        corners = [
            np.array([side/2, side/2, 0.0]),    # top-right
            np.array([-side/2, side/2, 0.0]),   # top-left
            np.array([-side/2, -side/2, 0.0]),  # bottom-left
            np.array([side/2, -side/2, 0.0]),   # bottom-right
            np.array([side/2, side/2, 0.0]),    # back to start
        ]
        
        # Interpola tra angoli
        for i in range(len(corners)-1):
            for j in range(num_points_per_side):
                alpha = j / num_points_per_side
                point = corners[i] + alpha * (corners[i+1] - corners[i])
                waypoints.append(point)
        
        return waypoints
    
    def run_test(self):
        """Esegue test completo"""
        
        # Attendi posizione iniziale
        if not self.wait_for_pose():
            return
        
        self.initial_position = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        
        self.get_logger().info('✅ Posizione iniziale acquisita')
        self.get_logger().info(
            f'   Posizione: [{self.initial_position[0]:.4f}, '
            f'{self.initial_position[1]:.4f}, {self.initial_position[2]:.4f}]'
        )
        self.get_logger().info('='*70)
        
        # Test 1: Quadrato XY
        waypoints = self.generate_square(side=0.05, num_points_per_side=8)
        self.execute_trajectory('Test 1: Quadrato piano XY (lato 5cm)', waypoints, 0.6)
        
        time.sleep(2.0)  # Pausa più lunga
        
        # Test 2: Cerchio XY
        waypoints = self.generate_circle_xy(radius=0.03, num_points=16)
        self.execute_trajectory('Test 2: Cerchio piano XY (r=3cm)', waypoints, 0.6)
        
        time.sleep(2.0)
        
        # Test 3: Cerchio XZ (verticale)
        waypoints = self.generate_circle_xz(radius=0.03, num_points=16)
        self.execute_trajectory('Test 3: Cerchio piano XZ verticale (r=3cm)', waypoints, 0.6)
        
        time.sleep(2.0)
        
        # Test 4: Figura a 8
        waypoints = self.generate_figure_eight(size=0.035, num_points=32)
        self.execute_trajectory('Test 4: Figura a otto (3.5cm)', waypoints, 0.4)
        
        time.sleep(2.0)
        
        # Test 5: Spirale 3D
        waypoints = self.generate_spiral_3d(radius=0.04, height=0.05, num_turns=1.5, num_points=24)
        self.execute_trajectory('Test 5: Spirale 3D (r=4cm, h=5cm)', waypoints, 0.5)
        
        time.sleep(2.0)
        
        # Test 6: Stella
        waypoints = self.generate_star_pattern(radius=0.04, num_rays=6)
        self.execute_trajectory('Test 6: Pattern stella (r=4cm, 6 raggi)', waypoints, 0.8)
        
        # Test completato
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('✅ TUTTI I TEST COMPLETATI')
        
        # Posizione finale vs iniziale
        if self.wait_for_pose():
            final_pos = np.array([
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z
            ])
            total_drift = final_pos - self.initial_position
            drift_magnitude = np.linalg.norm(total_drift)
            
            self.get_logger().info(
                f'   Drift totale: {drift_magnitude*1000:.1f}mm'
            )
            self.get_logger().info(
                f'   Δ finale: [{total_drift[0]:+.4f}, {total_drift[1]:+.4f}, '
                f'{total_drift[2]:+.4f}]'
            )
        
        self.get_logger().info('='*70)


def main(args=None):
    rclpy.init(args=args)
    
    test_node = ImpedanceComplexTest()
    
    # Attendi connessione
    time.sleep(1.0)
    
    try:
        test_node.run_test()
        time.sleep(1.0)
        
    except KeyboardInterrupt:
        test_node.get_logger().info('❌ Test interrotto!')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
