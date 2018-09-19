import sys, random

common_model_types = ["apartment",
	"house_1",
	"house_2",
	"house_3",
	"playground"
	]
rare_model_types = [	"fire_station",
	"grocery_store",
	"law_office",
	"office_building",
	"osrf_first_office",
	"post_office",
	"school",
	"cafe",
	"police_station",
	"gas_station",
	"fire_station"]
rare_model_distances = [80, 80, 20, 50, 50, 20, 30, 60, 30, 100, 80] 
common_model_distances = [30, 20, 20, 20,  20]

model_string = '''
<model name="building_{}">
  	<static>true</static>
  	<pose>{} {} 0 0 0 {}</pose>
  	<include>
    <uri>model://{}</uri>
  	</include>
</model>
'''	


max_length = int(sys.argv[1])
startX = int(sys.argv[2])
startY = int(sys.argv[3])
h_or_v = sys.argv[4]
u_or_d = sys.argv[5]
length = 0
posX = startX
posY = startY
index = 0
for i in range(2):
	while length < max_length:
		model_rand = random.randrange(5)
		if model_rand < 3:
			model_no = random.randrange(len(common_model_types))
			length = length + common_model_distances[model_no]
			if h_or_v == "H":
				print(model_string.format(index, posX, posY, 0, common_model_types[model_no]))

				if u_or_d == "P":
					posX = posX + common_model_distances[model_no]
				elif u_or_d == "N":
					posX = posX - common_model_distances[model_no]
			elif h_or_v == "V":
				print(model_string.format(index, posX, posY, 1.57 , common_model_types[model_no]))

				if u_or_d == "P":
					posY = posY + common_model_distances[model_no]
				elif u_or_d == "N":
					posY = posY - common_model_distances[model_no]	
		else:
			model_no = random.randrange(len(rare_model_types))
			length = length + rare_model_distances[model_no]
			if h_or_v == "H":
				print(model_string.format(index, posX, posY, 0, rare_model_types[model_no]))
				if u_or_d == "P":
					posX = posX + rare_model_distances[model_no]
				elif u_or_d == "N":
					posX = posX - rare_model_distances[model_no]
			elif h_or_v == "V":
				print(model_string.format(index, posX, posY, 1.57, rare_model_types[model_no]))		
				if u_or_d == "P":
					posY = posY + rare_model_distances[model_no]
				elif u_or_d == "N":
					posY = posY - rare_model_distances[model_no]	
		index = index + 1
	length = 0		
	if h_or_v == "H":
		if u_or_d == "P":
			posX = startX 
			posY = startY + (i+1) * 100
		elif u_or_d == "N":
			posX = startX 
			posY = startY - (i+1) * 100
	elif h_or_v == "V":
		if u_or_d == "P":
			posX = startX + (i+1) * 100
			posY = startY 
		elif u_or_d == "N": 	
			posX = startX - (i+1) * 100
			posY = startY 
