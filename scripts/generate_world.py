import math

def generate_circular_world():
    # --- Customization Variables ---
    radius = 3.0          # Distance from center to walls (meters)
    segments = 24         # Number of wall segments (higher = smoother circle)
    wall_height = 1.0     # Height of the walls (meters)
    wall_thickness = 0.15 # Thickness of the walls (meters)
    pole_radius = 0.4     # Radius of the center pole (meters)
    pole_height = 1.5     # Height of the center pole (meters)
    # -------------------------------
    
    # Start the SDF XML string
    xml = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="circular_world">
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    
    <model name="center_pole">
      <static>true</static>
      <pose>0 0 {pole_height/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{pole_radius}</radius><length>{pole_height}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{pole_radius}</radius><length>{pole_height}</length></cylinder></geometry>
          <material><script><name>Gazebo/Blue</name></script></material>
        </visual>
      </link>
    </model>
'''

    # Generate Circular Walls using trigonometry
    angle_step = 2 * math.pi / segments
    # Calculate the required length of each wall segment to form a closed loop
    wall_length = 2 * radius * math.tan(math.pi / segments) + (wall_thickness * 0.5)

    for i in range(segments):
        angle = i * angle_step
        # Calculate X and Y position for the center of this wall segment
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        
        xml += f'''
    <model name="wall_{i}">
      <static>true</static>
      <pose>{x} {y} {wall_height/2} 0 0 {angle}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{wall_thickness} {wall_length} {wall_height}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{wall_thickness} {wall_length} {wall_height}</size></box></geometry>
          <material><script><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>'''

    # Close the SDF XML string
    xml += '''
  </world>
</sdf>'''

    # Write to file
    filename = 'circular_room.world'
    with open(filename, 'w') as f:
        f.write(xml)
    print(f"Success! Generated '{filename}'")

if __name__ == '__main__':
    generate_circular_world()