import sys, random, math

common_model_types = ["apartment",
	"house_1",
	"house_2",
	"house_3"
	]
rare_model_types = ["fire_station",
	"grocery_store",
	"law_office",
	"office_building",
	"osrf_first_office",
	"post_office",
	"school",
	"cafe",
	"police_station",
	"gas_station"]
rare_model_distances_h = [80, 80, 20, 50, 50, 20, 30, 60, 30, 100] 
rare_model_distances_v = [30, 30, 20, 30, 30, 20, 30, 30, 30, 40] 
common_model_distances = [30, 25, 25, 25]

model_string = '''
<model name="building_{}">
	<static>true</static>
	<pose>{} {} 0 0 0 {}</pose>
	<include>
	<uri>model://{}</uri>
	</include>
</model>
'''	

def angle_street(max_length, startX, startY, angle):

	global index
	global output
	length = 0
	posX = startX
	posY = startY
	while length < max_length:
		model_no = random.randrange(len(common_model_types))
		length = length + common_model_distances[model_no]
		print(model_string.format(index, posX, posY, 0, common_model_types[model_no]))
		output.write(model_string.format(index, posX, posY, 0, common_model_types[model_no]))
		posX = posX + common_model_distances[model_no] * math.cos(math.radians(angle))
		posY = posY + common_model_distances[model_no] * math.sin(math.radians(angle))
		index = index + 1

def street(max_length, startX, startY, h_or_v, u_or_d):
	# max_length = int(sys.argv[1])
	# startX = int(sys.argv[2])
	# startY = int(sys.argv[3])
	# h_or_v = sys.argv[4]
	# u_or_d = sys.argv[5]
	global index
	global output
	length = 0
	posX = startX
	posY = startY

	while length < max_length:
		model_rand = random.randrange(8)
		if model_rand < 6:
			model_no = random.randrange(len(common_model_types))
			length = length + common_model_distances[model_no]
			if h_or_v == "H":
				print(model_string.format(index, posX, posY, 0, common_model_types[model_no]))
				output.write(model_string.format(index, posX, posY, 0, common_model_types[model_no]))
				if u_or_d == "P":
					posX = posX + common_model_distances[model_no]
				elif u_or_d == "N":
					posX = posX - common_model_distances[model_no]
			elif h_or_v == "V":
				print(model_string.format(index, posX, posY, 1.57 , common_model_types[model_no]))
				output.write(model_string.format(index, posX, posY, 1.57 , common_model_types[model_no]))
				if u_or_d == "P":
					posY = posY + common_model_distances[model_no]
				elif u_or_d == "N":
					posY = posY - common_model_distances[model_no]	
		else:
			model_no = random.randrange(len(rare_model_types))
			if h_or_v == "H":
				length = length + rare_model_distances_h[model_no]
				print(model_string.format(index, posX, posY, 0, rare_model_types[model_no]))
				output.write(model_string.format(index, posX, posY, 0, rare_model_types[model_no]))
				if u_or_d == "P":
					posX = posX + rare_model_distances_h[model_no]
				elif u_or_d == "N":
					posX = posX - rare_model_distances_h[model_no]
			elif h_or_v == "V":
				length = length + rare_model_distances_v[model_no]
				print(model_string.format(index, posX, posY, 1.57, rare_model_types[model_no]))		
				output.write(model_string.format(index, posX, posY, 1.57, rare_model_types[model_no]))
				if u_or_d == "P":
					posY = posY + rare_model_distances_v[model_no]
				elif u_or_d == "N":
					posY = posY - rare_model_distances_v[model_no]	
		index = index + 1
		
def square(startX, startY, length):
	street(length, startX, startY, "H", "P")
	street(length, startX, startY-length, "H", "P")
	street(length, startX, startY, "V", "N")
	street(length, startX+length, startY, "V", "N")

index = 0
output = open("buildings2.txt", "w")
	
# square(-100,100, 200)

# square(-160,160, 320)
	
# square(-230,230, 460)

# square(-330, 330, 660)

# square(-450, 450, 900)


# street(400, -50, 50, "V", "P")	

# street(400, -50, 50, "H", "N")

# street(300, -150, 150, "V", "P")	

# street(300, -150, 150, "H", "N")

# street(200, -250, 250, "V", "P")	

# street(200, -250, 250, "H", "N")



# street(400, 50, -50, "V", "N")	

# street(400, 50, -50, "H", "P")

# street(300, 150, -150, "V", "N")	

# street(300, 150, -150, "H", "P")

# street(200, 250, -250, "V", "N")	

# street(200, 250, -250, "H", "P")

# street(400, 50, 50 , "H", "P")
# street(400, 50, 200 , "H", "P")
# street(400, 50, 350 , "H", "P")

# street(400, -50, -50 , "V", "N")
# street(400, -200, -50 , "V", "N")
# street(400, -350, -50 , "V", "N")




# square(-100, 100, 200)

# street(70, -170, 170, "H", "N")
# street(70, -170, 170, "V", "P")

# street(70, 170, 170, "H", "P")
# street(70, 170, 170, "V", "P")

# street(70, 170, -170, "H", "P")
# street(70, 170, -170, "V", "N")

# street(70, -170, -170, "H", "N")
# street(70, -170, -170, "V", "N")

# street(70, -170, 100, "H", "N")
# street(70, -170, -100, "H", "N")
# street(190, -170, 100, "V", "N")

# street(70, 170, 100, "H", "P")
# street(70, 170, -100, "H", "P")
# street(190, 170, 100, "V", "N")

# street(70, -100, 170, "V", "P")
# street(70, 100, 170, "V", "P")
# street(190, -100, 170, "H", "P")

# street(70, -100, -170, "V", "N")
# street(70, 100, -170, "V", "N")
# street(190, -100, 170, "H", "P")



# street(420, 170, 195, "H", "N")
# street(80, 170, 195, "V", "P")


# street(420, 170, -195, "H", "N")
# street(80, 170, -195, "V", "N")

# street(420, 170, 25, "H", "N")
# street(420, 170, -25, "H", "N")
# output.write(model_string.format(index, 170, 0, 0, "house_1"))
# index = index + 1 

# street(420, -170, 85, "H", "P")
# street(420, -170, 135, "H", "P")
# output.write(model_string.format(index, -170, 110, 0, "house_2"))
# index = index + 1 

# street(420, -170, -85, "H", "P")
# street(420, -170, -135, "H", "P")
# output.write(model_string.format(index, -170, -100, 0, "house_3"))
# index = index + 1 



angle_street(80, -220, 210, 0)
angle_street(85, -220, 210, -90)
angle_street(80, -220, 120, 0)
angle_street(85, -140, 120, -90)
angle_street(80, -140, 30, 180)


angle_street(180, -100, 30, 90)
angle_street(80, -100, 30, 0)
angle_street(180, -20, 30, 90)

angle_street(180, 20, 30, 90)
angle_street(80, 20, 210, 0)
angle_street(90, 100, 210, -90)
angle_street(80, 100, 120, 180)
angle_street(120, 20, 120, -45)

angle_street(180, 140, 30, 90)
angle_street(80, 140, 30, 0)
angle_street(180, 220, 30, 90)


angle_street(180, -130, -30, -90)


angle_street(180, 0, -30, -90)
angle_street(80, 0, -120, 180)
angle_street(180, -80, -30, -90)

angle_street(190, 100, -30, 240)
angle_street(190, 100, -30, -60)
angle_street(50, 75, -120, 0)

output.close()	