import random
import math
import pyproj
import chevron
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
    
    
    # f.write("\t\t\t<ode>\n")
    # f.write("\t\t\t\t<mu>0.00000</mu>\n")
    # f.write("\t\t\t\t<kp>0.00000</kp>\n")
    # f.write("\t\t\t\t<kd>0.00000</kd>\n")
    # f.write("\t\t\t</ode>\n")
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

