import math 

# Get the box structure steps for numerator / denominator
# returns (spoken steps, tablet_steps, all_answers) where
# spoken_steps = a list of utterances for the robot to explain the problem
# tablet_steps = a list of the boxes to fill in each step in the form "line-column-digit:line-column-digit", "line-column-digit", ... 
# all answers = a string containing all of the intermediate answers (in the same form as above)
def get_box_steps(numerator, denominator):
	# find length of numerator
	places = 0
	while (numerator > math.pow(10, places)):
		places += 1

	first_numerator = numerator
	for i in range (places-1):
		first_numerator = first_numerator / 10

	if (first_numerator < denominator):
		places -= 1

	spoken_text = ["Let's try to do this similar type of problem together! What is " + str(numerator) + " divided by " +str(denominator) + "?"]
	answer = "The answer is " + str(numerator/denominator)
	if (numerator % denominator !=0):
		answer += " remainder "
		answer += str(numerator % denominator)
	answer += ". Now try the problem you were working on before"

	(robot_speech, tablet_steps) = rec_get_boxes(1, 1, numerator, denominator, places)
 	spoken_text.extend(robot_speech)
 	spoken_text.append(answer)
	all_answers = (":").join(tablet_steps)
	tablet_steps.insert(0,"") 

	return (spoken_text, tablet_steps, all_answers)

# the recursive function to build up the steps
def rec_get_boxes(line, box, numerator, denominator, digits_left):

	print "in rec boxes line: " + str(line) + " box " + str(box) + " numerator: " + str(numerator) + " denominator " + str(denominator) + "digits: " + str(digits_left)

	spoken_text = []
	tablet_steps = []

	if (digits_left == 0): 	
		print "done"			
		return (spoken_text, tablet_steps)

	step_numerator = numerator
	print "step numerator: " + str(step_numerator)

	for i in range (digits_left-1):
		step_numerator = step_numerator / 10

	rest = int(numerator - math.pow(10, digits_left-1) * step_numerator)

	ans_box = box
	if (step_numerator > 10 and step_numerator / 10 > denominator):
		ans_box += 1
		digits_left -= 1
	elif (step_numerator > 10):
		ans_box += 1

	print "numerator for step: ", step_numerator, " rest is ", rest

	step_quotient = step_numerator / denominator
	step_remainder = step_numerator % denominator
	step_product = step_quotient * denominator

	## FIRST STEP: find quotient and subtract to get remainder

	# split step_product into boxes
	# if the step_numerator is more two digits, fill in boxes one and two
	if (step_numerator > 10):
		step_boxes = str(line) + "-" + str(box) + "-" + str(step_product/10) 
		step_boxes_part = ":" + str(line) + "-" + str(box + 1) + "-" + str(step_product%10) 
		step_boxes += step_boxes_part
	else: # otherwise, the entire step product goes in box one
		step_boxes = str(line) + "-" + str(box) + "-" + str(step_product) 

	# get remainder -- same as above, fill in one or two boxes depending on size of numerator
	if (step_numerator > 10):
		step_boxes_part = ":" + str(line+1) + "-" + str(box) + "-" + str(step_remainder/10) 
		step_boxes += step_boxes_part
		step_boxes_part = ":" + str(line+1) + "-" + str(box+1) + "-" + str(step_remainder%10) 
		step_boxes += step_boxes_part
		next_box = box + 1
	else: 
		step_boxes_part = ":" + str(line+1) + "-" + str(box) + "-" + str(step_remainder) 
		step_boxes += step_boxes_part
		next_box = box 

	# put answer in boxes
	step_boxes_part = ":0-" + str(ans_box) + "-" + str(step_quotient)
	step_boxes+=step_boxes_part

	# form spoken text
	step_text = str(denominator) + " goes into " +  str(step_numerator) + ". " + str(step_quotient) + " times "
	step_text_part2 = " so we can subtract " + str(step_product) + " from " + str(step_numerator) 
	step_text += step_text_part2
	step_text_part2 = ". This gives us " + str(step_remainder) + ". "
	step_text += step_text_part2

	spoken_text.append(step_text)
	tablet_steps.append(step_boxes)

	## SECOND STEP: pull down next digit from numerator
	print "digits_left", digits_left
	if (digits_left > 1):
		pull_down = rest
		if (pull_down < math.pow(10, digits_left-2)):
			pull_down = 0
			pull_down_place = 0
		else:
			pull_down_place = 0
			while (pull_down > 10):
				pull_down_place += 1
				pull_down = pull_down / 10

		print "pull_down_place " + str(pull_down_place) + " pull_down: " + str(pull_down)

		# remainder and rest is next numerator
		next_numerator = int(math.pow(1, pull_down_place) * rest + math.pow(10, digits_left-1) * step_remainder)

		next_step_numerator = next_numerator
		for i in range (digits_left-2):
			next_step_numerator = next_step_numerator / 10
		print next_step_numerator, " is next_step_numerator"
		if (next_step_numerator < 10):
			next_box = box + 1
		
		step_text = "Now we can bring down the " + str(pull_down) + " from the dividend. Then, for the next step we have " + str(next_step_numerator) + "."
		if (step_numerator > 10):
			step_boxes =  str(line+1) + "-" + str(box+2) + "-" + str(pull_down) 
		else:
			step_boxes =  str(line+1) + "-" + str(box+1) + "-" + str(pull_down) 

		spoken_text.append(step_text)
		tablet_steps.append(step_boxes)

		(rec_speech, rec_steps) = rec_get_boxes(line+2, next_box, next_numerator, denominator, digits_left - 1)
		spoken_text.extend(rec_speech)
		tablet_steps.extend(rec_steps)

		return (spoken_text, tablet_steps)

	else :
		## END of Numerator
		if (step_remainder != 0):
			step_text = "The remainder is " + str(step_remainder) + "."
			step_boxes = "0-" + str(ans_box +1) + "-R:0-" + str(ans_box+2) + "-" + str(step_remainder) 
			tablet_steps.append(step_boxes)

		else:
			step_text = "There are no more digits left to bring down."

		print "digits left " + str(digits_left) + " this is the end"

		spoken_text.append(step_text)

		return (spoken_text, tablet_steps)



	