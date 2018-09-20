import sys, random

common_model_types = ["apartment",
	"house_1",
	"house_2",
	"house_3",
	"playground"
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
common_model_distances = [30, 25, 25, 25,  25]

model_string = '''
<model name="building_{}">
	<static>true</static>
	<pose>{} {} 0 0 0 {}</pose>
	<include>
	<uri>model://{}</uri>
	</include>
</model>
'''	


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
output = open("buildings.txt", "w")

square(-100,100, 200)

square(-160,160, 320)
	
square(-230,230, 460)

square(-330, 330, 660)

square(-450, 450, 900)
	

output.close()	