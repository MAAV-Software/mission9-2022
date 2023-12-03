import random
import math
import pyproj
starting_coord = (0, 0, 0) # degree starting position plus height
# total_way_points = 10
total_distance = 3/69 # 3 miles in degrees (1 degree = 69 miles)
minHeight = 100 # agl or would be 217 feet msl
maxHeight = 125
way_points = []
way_points.append(starting_coord)

#generate a random number between 5-10
howMany = random.randint(5, 10)
step = total_distance/howMany
variance = 0.00001
distance = 0
for i in range(1, howMany+1): #run through are total points
    # x = random.uniform(-total_distance/2, total_distance/2) #generate a random x and y coordinate that is within the total distance remaining
    # y = random.uniform(-total_distance/2, total_distance/2)
    radius = step
    angle = random.uniform(0, 2*3.14159)
    long = way_points[i-1][0]+ (radius * math.cos(angle))
    lat =  way_points[i-1][1]+ radius * math.sin(angle)

    height = random.uniform(minHeight, maxHeight)
    way_points.append((long, lat, height))
    distance += (((abs(long)-abs(way_points[i-1][0]))**2+(abs(lat)-abs(way_points[i-1][1]))**2)**0.5) #subtract the distance between the two points from the total distance so that we stay within 10
    # print(total_distance*69)
# print(total_distance*69)
# print(distance*69)
#we're going to create an sdf file with the points
# <spherical_coordinates>
#   <surface_model>EARTH_WGS84</surface_model>
#   <world_frame_orientation>ENU</world_frame_orientation>
#   <latitude_deg>-22.9</latitude_deg>
#   <longitude_deg>-43.2</longitude_deg>
#   <elevation>0</elevation>
#   <heading_deg>0</heading_deg>
# </spherical_coordinates>



# <light name='user_way_Point_0' type='point'>
#       <pose>2.17619 -2.70115 1 0 -0 0</pose>
#       <diffuse>0.5 0.5 0.5 1</diffuse>
#       <specular>0.1 0.1 0.1 1</specular>
#       <attenuation>
#         <range>20</range>
#         <constant>0.5</constant>
#         <linear>0.01</linear>
#         <quadratic>0.001</quadratic>
#       </attenuation>
#       <cast_shadows>0</cast_shadows>
#       <direction>0 0 -1</direction>
#       <spot>
#         <inner_angle>0</inner_angle>
#         <outer_angle>0</outer_angle>
#         <falloff>0</falloff>
#       </spot>
#     </light>
f = open("way_points.txt", "w")
f.write("Total path length: " + str(distance*69) + " miles\n")
f.write("Total points: " + str(len(way_points)) + "\n")
for i in range(0, len(way_points)):
    f.write(str(way_points[i][0]) + " " + str(way_points[i][1]) + " " + str(way_points[i][2]) + "\n")
f.close()


f = open("waypoints.world", "w")
f.write("<sdf version='1.7'>\n")
f.write("<world name='default'>\n")

#print out the sdf file above in proper format
f.write("<light name='sun' type='directional'>\n")
f.write("\t<cast_shadows>1</cast_shadows>\n")
f.write("\t<pose>0 0 10 0 -0 0</pose>\n")
f.write("\t<diffuse>0.8 0.8 0.8 1</diffuse>\n")
f.write("\t<specular>0.2 0.2 0.2 1</specular>\n")
f.write("\t<attenuation>\n")
f.write("\t\t<range>1000</range>\n")
f.write("\t\t<constant>0.9</constant>\n")
f.write("\t\t<linear>0.01</linear>\n")
f.write("\t\t<quadratic>0.001</quadratic>\n")
f.write("\t</attenuation>\n")
f.write("\t<direction>-0.5 0.1 -0.9</direction>\n")
f.write("\t<spot>\n")
f.write("\t\t<inner_angle>0</inner_angle>\n")
f.write("\t\t<outer_angle>0</outer_angle>\n")
f.write("\t\t<falloff>0</falloff>\n")
f.write("\t</spot>\n")
f.write("</light>\n")
f.write("<model name='ground_plane'>\n")
f.write("\t<static>1</static>\n")
f.write("\t<link name='link'>\n")
f.write("\t\t<collision name='collision'>\n")
f.write("\t\t\t<geometry>\n")
f.write("\t\t\t\t<plane>\n")
f.write("\t\t\t\t\t<normal>0 0 1</normal>\n")
f.write("\t\t\t\t\t<size>100 100</size>\n")
f.write("\t\t\t\t</plane>\n")
f.write("\t\t\t</geometry>\n")
f.write("\t\t\t<surface>\n")
f.write("\t\t\t\t<contact>\n")
f.write("\t\t\t\t\t<collide_bitmask>65535</collide_bitmask>\n")
f.write("\t\t\t\t\t<ode/>\n")
f.write("\t\t\t\t</contact>\n")
f.write("\t\t\t\t<friction>\n")
f.write("\t\t\t\t\t<ode>\n")
f.write("\t\t\t\t\t\t<mu>100</mu>\n")
f.write("\t\t\t\t\t\t<mu2>50</mu2>\n")
f.write("\t\t\t\t\t</ode>\n")
f.write("\t\t\t\t\t<torsional>\n")
f.write("\t\t\t\t\t\t<ode/>\n")
f.write("\t\t\t\t\t</torsional>\n")
f.write("\t\t\t\t</friction>\n")
f.write("\t\t\t\t<bounce/>\n")
f.write("\t\t\t</surface>\n")
f.write("\t\t\t<max_contacts>10</max_contacts>\n")
f.write("\t\t</collision>\n")
f.write("\t\t<visual name='visual'>\n")
f.write("\t\t\t<cast_shadows>0</cast_shadows>\n")
f.write("\t\t\t<geometry>\n")
f.write("\t\t\t\t<plane>\n")
f.write("\t\t\t\t\t<normal>0 0 1</normal>\n")
f.write("\t\t\t\t\t<size>100 100</size>\n")
f.write("\t\t\t\t</plane>\n")
f.write("\t\t\t</geometry>\n")
f.write("\t\t\t<material>\n")
f.write("\t\t\t\t<script>\n")
f.write("\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n")
f.write("\t\t\t\t\t<name>Gazebo/Grey</name>\n")
f.write("\t\t\t\t</script>\n")
f.write("\t\t\t</material>\n")
f.write("\t\t</visual>\n")
f.write("\t\t<self_collide>0</self_collide>\n")
f.write("\t\t<enable_wind>0</enable_wind>\n")
f.write("\t\t<kinematic>0</kinematic>\n")
f.write("\t</link>\n")
f.write("</model>\n")
f.write("<model name='asphalt_plane'>\n")
f.write("\t<static>1</static>\n")
f.write("\t<link name='link'>\n")
f.write("\t\t<collision name='collision'>\n")
f.write("\t\t\t<geometry>\n")
f.write("\t\t\t\t<box>\n")
f.write("\t\t\t\t\t<size>200 200 0.1</size>\n")
f.write("\t\t\t\t</box>\n")
f.write("\t\t\t</geometry>\n")
f.write("\t\t\t<max_contacts>10</max_contacts>\n")
f.write("\t\t\t<surface>\n")
f.write("\t\t\t\t<contact>\n")
f.write("\t\t\t\t\t<ode/>\n")
f.write("\t\t\t\t</contact>\n")
f.write("\t\t\t\t<bounce/>\n")
f.write("\t\t\t\t<friction>\n")
f.write("\t\t\t\t\t<torsional>\n")
f.write("\t\t\t\t\t\t<ode/>\n")
f.write("\t\t\t\t\t</torsional>\n")
f.write("\t\t\t\t\t<ode/>\n")
f.write("\t\t\t\t</friction>\n")
f.write("\t\t\t</surface>\n")
f.write("\t\t</collision>\n")
f.write("\t\t<visual name='visual'>\n")
f.write("\t\t\t<cast_shadows>0</cast_shadows>\n")
f.write("\t\t\t<geometry>\n")
f.write("\t\t\t\t<box>\n")
f.write("\t\t\t\t\t<size>200 200 0.1</size>\n")
f.write("\t\t\t\t</box>\n")
f.write("\t\t\t</geometry>\n")
f.write("\t\t\t<material>\n")
f.write("\t\t\t\t<script>\n")
f.write("\t\t\t\t\t<uri>model://asphalt_plane/materials/scripts</uri>\n")
f.write("\t\t\t\t\t<uri>model://asphalt_plane/materials/textures</uri>\n")
f.write("\t\t\t\t\t<name>vrc/asphalt</name>\n")
f.write("\t\t\t\t</script>\n")
f.write("\t\t\t</material>\n")
f.write("\t\t</visual>\n")
f.write("\t\t<self_collide>0</self_collide>\n")
f.write("\t\t<enable_wind>0</enable_wind>\n")
f.write("\t\t<kinematic>0</kinematic>\n")
f.write("\t</link>\n")
f.write("</model>\n")
f.write("<physics name='default_physics' default='0' type='ode'>\n")
f.write("\t<ode>\n")
f.write("\t\t<solver>\n")
f.write("\t\t\t<type>quick</type>\n")
f.write("\t\t\t<iters>10</iters>\n")
f.write("\t\t\t<sor>1.3</sor>\n")
f.write("\t\t\t<use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>\n")
f.write("\t\t</solver>\n")
f.write("\t\t<constraints>\n")
f.write("\t\t\t<cfm>0</cfm>\n")
f.write("\t\t\t<erp>0.2</erp>\n")
f.write("\t\t\t<contact_max_correcting_vel>100</contact_max_correcting_vel>\n")
f.write("\t\t\t<contact_surface_layer>0.001</contact_surface_layer>\n")
f.write("\t\t</constraints>\n")
f.write("\t</ode>\n")
f.write("\t<max_step_size>0.004</max_step_size>\n")
f.write("\t<real_time_factor>1</real_time_factor>\n")
f.write("\t<real_time_update_rate>250</real_time_update_rate>\n")
f.write("</physics>\n")
f.write("<gravity>0 0 -9.8066</gravity>\n")
f.write("<magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>\n")
f.write("<atmosphere type='adiabatic'/>\n")
f.write("<scene>\n")
f.write("\t<ambient>0.4 0.4 0.4 1</ambient>\n") 
f.write("\t<background>0.7 0.7 0.7 1</background>\n")
f.write("\t<shadows>1</shadows>\n")
f.write("</scene>\n")
f.write("<wind/>\n")
f.write("<spherical_coordinates>\n")
f.write("\t<surface_model>EARTH_WGS84</surface_model>\n")
f.write("\t<world_frame_orientation>ENU</world_frame_orientation>\n")
f.write("\t<latitude_deg>0</latitude_deg>\n")
f.write("\t<longitude_deg>0</longitude_deg>\n")
f.write("\t<elevation>0</elevation>\n")
f.write("\t<heading_deg>0</heading_deg>\n")
f.write("</spherical_coordinates>\n")



source_proj = pyproj.Proj(init='epsg:4326')  # WGS84
target_proj = pyproj.Proj(init='epsg:3857') 


for i in range(0, len(way_points)):
    f.write("<model name='user_way_Point_" + str(i) + "'>\n")
    f.write("\t<static>1</static>\n")
    # f.write("\t<pose>" + str(25) + " " + str(25) + " " + str(25) + " 0 -0 0</pose>\n")
    #project the lat and long to x and y using WGS84 projection
    
    x, y = pyproj.transform(source_proj, target_proj, way_points[i][0], way_points[i][1])
    f.write("\t<pose>" + str(x) + " " + str(y) + " " + str(way_points[i][2]) + " 0 -0 0</pose>\n")
    # f.write("\t<pose>" + str(way_points[i][0]*111194.9) + " " + str(way_points[i][1]*111194.9) + " " + str(way_points[i][2]) + " 0 -0 0</pose>\n")

    f.write("<link name='link'>\n")
    f.write("<gravity>0</gravity>\n")
    f.write("\t<inertial>\n")
    f.write("\t<mass>1</mass>\n")
    f.write("\t<inertia>\n")
    f.write("\t\t<ixx>0.0001</ixx>\n")
    f.write("\t\t<ixy>0</ixy>\n")
    f.write("\t\t<ixz>0</ixz>\n")
    f.write("\t\t<iyy>0.0001</iyy>\n")
    f.write("\t\t<iyz>0</iyz>\n")
    f.write("\t\t<izz>0.0001</izz>\n")
    f.write("\t</inertia>\n")
    f.write("<pose> 0 0 0 0 -0 0</pose>\n")
    f.write("</inertial>\n")
    f.write("<collision name='collision'>\n")
    f.write("\t<geometry>\n")
    f.write("\t\t<sphere>\n")
    f.write("\t\t\t<radius>7.62</radius>\n")
    f.write("\t\t</sphere>\n")
    f.write("\t</geometry>\n")
    f.write("\t<surface>\n")
    f.write("\t\t<contact>\n")
    f.write("\t\t\t<collide_without_contact>1</collide_without_contact>\n")
    # f.write("\t\t\t<ode>\n")
    # f.write("\t\t\t\t<mu>0.00000</mu>\n")
    # f.write("\t\t\t\t<kp>0.00000</kp>\n")
    # f.write("\t\t\t\t<kd>0.00000</kd>\n")
    # f.write("\t\t\t</ode>\n")
    f.write("\t\t</contact>\n")
    # f.write("\t\t<bounce>\n")
    # f.write("\t\t\t<restitution_coefficient>0.00000</restitution_coefficient>\n")
    # f.write("\t\t\t<threshold>0.00000</threshold>\n")
    # f.write("\t\t</bounce>\n")
    # f.write("\t\t<friction>\n")
    # f.write("\t\t\t<torsional>\n")
    # f.write("\t\t\t\t<coefficient>0<\coefficient>\n")
    # f.write("\t\t\t\t<ode/>\n")
    # f.write("\t\t\t</torsional>\n")
    # f.write("\t\t\t<ode>\n")
    # f.write("\t\t\t<mu>0.00000</mu>\n")
    # f.write("\t\t\t<mu2>0.00000</mu2>\n")
    # f.write("\t\t\t</ode>\n")
    # f.write("\t\t</friction>\n")
    f.write("\t</surface>\n")
    f.write("</collision>\n")
    f.write("<visual name='visual'>\n")
    f.write("\t<geometry>\n")
    f.write("\t\t<sphere>\n")
    f.write("\t\t\t<radius>7.62</radius>\n")
    f.write("\t\t</sphere>\n")
    f.write("\t</geometry>\n")
    f.write("\t<material>\n")
    f.write("\t\t<script>\n")
    f.write("\t\t\t<name>Gazebo/Blue</name>\n")
    f.write("\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n")
    f.write("\t\t</script>\n")
    f.write("\t</material>\n")
    f.write("</visual>\n")
    f.write("<self_collide>0</self_collide>\n")
    f.write("<enable_wind>0</enable_wind>\n")
    f.write("<kinematic>0</kinematic>\n")
    f.write("</link>\n")
    f.write("</model>\n")




    # <spherical_coordinates>
    #   <surface_model>EARTH_WGS84</surface_model>
    #   <latitude_deg>0</latitude_deg>
    #   <longitude_deg>0</longitude_deg>
    #   <elevation>0</elevation>
    #   <heading_deg>0</heading_deg>
    # </spherical_coordinates>




#     '<link name=\"link\">'\
# '<visual name=\"visual\">'\
# '<geometry><sphere><radius>1.0</radius></sphere></geometry>'\
# '</visual>'\
# '<collision name=\"visual\">'\
# '<geometry><sphere><radius>1.0</radius></sphere></geometry>'\
# '</collision>'\
# '</link>'\
    # f.write("\t<link name='link'>\n")
    # f.write("\t\t<visual name='visual'>\n")
    # f.write("\t\t\t<geometry>\n")
    # f.write("\t\t\t\t<sphere>\n")
    # f.write("\t\t\t\t\t<radius>1.0</radius>\n")
    # f.write("\t\t\t\t</sphere>\n")
    # f.write("\t\t\t</geometry>\n")
    # f.write("\t\t</visual>\n")
    # f.write("\t\t<collision name='collision'>\n")
    # f.write("\t\t\t<geometry>\n")
    # f.write("\t\t\t\t<sphere>\n")
    # f.write("\t\t\t\t\t<radius>25.0</radius>\n")
    # f.write("\t\t\t\t</sphere>\n")
    # f.write("\t\t\t</geometry>\n")
    # f.write("\t\t</collision>\n")
    # f.write("\t</link>\n")




    # f.write("\t<size>0</size>\n")

    # f.write("\t<pose>" + str(way_points[i][0]*111194.9) + " " + str(way_points[i][1]*111194.9) + " " + str(way_points[i][2]) + " 0 -0 0</pose>\n")
    # f.write("\t<diffuse>0.5 0.5 0.5 1</diffuse>\n")
    # f.write("\t<specular>0.1 0.1 0.1 1</specular>\n")
    # f.write("\t<attenuation>\n")
    # f.write("\t\t<range>20</range>\n")
    # f.write("\t\t<constant>0.5</constant>\n")
    # f.write("\t\t<linear>0.01</linear>\n")
    # f.write("\t\t<quadratic>0.001</quadratic>\n")
    # f.write("\t</attenuation>\n")
    # f.write("\t<cast_shadows>0</cast_shadows>\n")
    # f.write("\t<direction>0 0 -1</direction>\n")
    # f.write("\t<spot>\n")
    # f.write("\t\t<inner_angle>0</inner_angle>\n")   
    # f.write("\t\t<outer_angle>0</outer_angle>\n")
    # f.write("\t\t<falloff>0</falloff>\n")
    # f.write("\t</spot>\n")
    # f.write("</model>\n")
    
# f.write(str(way_points))
# print(way_points)
f.write("</world>\n")
f.write("</sdf>\n")