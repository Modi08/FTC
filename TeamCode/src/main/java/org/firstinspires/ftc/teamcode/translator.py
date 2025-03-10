file = open("turtle.txt", "r")
output = open("Real.txt", "w")

for line in file:
    print(line)
    command = line.split(".")[1].split("(", ")")
    
    if command[0].lower == "forward":
        output.write(f"moveBySetDistance({abs(int(command[1])), {"false" if int(command[1]) > 0 else "true"}})")
    elif command[1].lower == "left":
        output.write(f"turnRobot({int(command[1])}, false)")
    elif command[1].lower == "left":
        output.write(f"turnRobot({int(command[1])}, true)")
    else:
        output.write("command not know")