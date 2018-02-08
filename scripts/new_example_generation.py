import math 




def get_box_steps(numerator, denominator):

    tablet_steps = []
    spoken_text = []

    # add in the introductory text
    spoken_text.append("Let's try to do this similar type of problem together! What is " + str(numerator) + " divided by " +str(denominator) + "?")
    tablet_steps.append("")

    num_loops = len(str(numerator))
    # if the denominator doesn't go into the first digit of the numerator
    if (int(str(numerator)[0:1]) < denominator):
        num_loops -= 1
        step_numerator = int(str(numerator)[0:2])
        num_step_numerator_digits = 2
    else:
        step_numerator = int(str(numerator)[0:1])
        num_step_numerator_digits = 1

    step_quotient = step_numerator / denominator 
    step_product = denominator * step_quotient
    step_remainder = step_numerator - step_product

    current_row = 1
    current_column = 1

    # print "\n%i / %i\n" % (numerator, denominator)

    for i in range(num_loops):

        # print "Loop %i:\n\tstep_numerator: %i\n\tstep_quotient: %i\n\tstep_product: %i\n\tstep_remainder: %i\n" % (i, step_numerator, step_quotient, step_product, step_remainder)
        
        temp_tablet_step = ""
        
        if (num_step_numerator_digits == 1):
            temp_tablet_step += str(current_row) + "-" + str(current_column) + "-" + str(step_product)
            temp_tablet_step += ":" + str(current_row + 1) + "-" + str(current_column) + "-" + str(step_remainder)
            temp_tablet_step += ":" + "0" + "-" + str(current_column) + "-" + str(step_quotient)
        elif (num_step_numerator_digits == 2):
            # check to see if the product is 1 or 2 digits, pad with 0 if it's 1 digit
            if (len(str(step_product)) == 1):
                temp_tablet_step += str(current_row) + "-" + str(current_column) + "-" + "0"
                temp_tablet_step += ":" + str(current_row) + "-" + str(current_column + 1) + "-" + str(step_product)
            elif (len(str(step_product)) == 2):
                temp_tablet_step += str(current_row) + "-" + str(current_column) + "-" + str(step_product)[0]
                temp_tablet_step += ":" + str(current_row) + "-" + str(current_column + 1) + "-" + str(step_product)[1]
            else: 
                print "Error: step_product should be 1 or 2 digits"
            temp_tablet_step += ":" + str(current_row + 1) + "-" + str(current_column) + "-" + "0"
            temp_tablet_step += ":" + str(current_row + 1) + "-" + str(current_column + 1) + "-" + str(step_remainder)
            temp_tablet_step += ":" + "0" + "-" + str(current_column + 1) + "-" + str(step_quotient)
        else: 
            print "Error: num_step_numerator_digits should be 1 or 2"

        # append the step to the tablet step list
        tablet_steps.append(temp_tablet_step)

        temp_spoken_text = str(denominator) + " goes into " +  str(step_numerator) + ". " + str(step_quotient) + " times "
        temp_spoken_text += " so we can subtract " + str(step_product) + " from " + str(step_numerator) 
        temp_spoken_text += ". This gives us " + str(step_remainder) + ". "

        # append the spoken text to the list
        spoken_text.append(temp_spoken_text)

        temp_tablet_step = ""

        # if we're on our last loop - put the remainder next to the quotient if there is a remainder
        if (i == (num_loops - 1) and step_remainder != 0):
            if (num_step_numerator_digits == 1):
                temp_tablet_step += "0" + "-" + str(current_column + 1) + "-" + "R"
                temp_tablet_step += ":" + "0" + "-" + str(current_column + 2) + "-" + str(step_remainder)
            elif (num_step_numerator_digits == 2):
                temp_tablet_step += "0" + "-" + str(current_column + 2) + "-" + "R"
                temp_tablet_step += ":" + "0" + "-" + str(current_column + 3) + "-" + str(step_remainder)
            else: 
                print "Error: num_step_numerator_digits should be 1 or 2"
            temp_spoken_text = "The remainder is " + str(step_remainder) + "."
            tablet_steps.append(temp_tablet_step)
        # if we're not, pull down the next digit
        elif(i != (num_loops - 1)):
            if (num_step_numerator_digits == 1):
                if (num_loops-i-2 > 0):
                    pull_down = str(numerator)[-(num_loops-i-1):-(num_loops-i-2)]
                else: 
                    pull_down = str(numerator)[-(num_loops-i-1):]
                temp_tablet_step += str(current_row + 1) + "-" + str(current_column + 1) + "-" + pull_down
            elif (num_step_numerator_digits == 2):
                if (num_loops-i-2 > 0):
                    pull_down = str(numerator)[-(num_loops-i-1):-(num_loops-i-2)]
                else: 
                    pull_down = str(numerator)[-(num_loops-i-1):]
                temp_tablet_step += str(current_row + 1) + "-" + str(current_column + 2) + "-" + pull_down
            else: 
                pull_down = ""
                print "Error: num_step_numerator_digits should be 1 or 2"

            next_step_numerator = int(str(step_remainder) + pull_down)
            temp_spoken_text = "Now we can bring down the " + str(pull_down) + " from the dividend. Then, for the next step we have " + str(next_step_numerator) + "."
            tablet_steps.append(temp_tablet_step)

            # set up for the next loop
            step_numerator = next_step_numerator
            step_quotient = step_numerator / denominator 
            step_product = denominator * step_quotient
            step_remainder = step_numerator - step_product
            if (num_step_numerator_digits == 2):
                current_column += 1
            current_row += 2
            num_step_numerator_digits = 2


        # we're done remainder == 0
        else: 
            temp_spoken_text = "There are no more digits left to bring down."

        spoken_text.append(temp_spoken_text)


    answer = "The answer is " + str(numerator/denominator)
    if (numerator % denominator !=0):
        answer += " remainder "
        answer += str(numerator % denominator)
    answer += ". Now try the problem you were working on before"
    spoken_text.append(answer)

    all_answers = (":").join(tablet_steps[1:])

    return (spoken_text, tablet_steps, all_answers)



# if __name__ == "__main__": 

#     get_box_steps(348, 2)
#     get_box_steps(554, 2)
#     get_box_steps(141, 2)
#     get_box_steps(1357, 2)
#     get_box_steps(9600, 4)















