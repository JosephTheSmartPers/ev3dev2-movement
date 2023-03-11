def split(word):
    return [char for char in word]
def switch(word):

    return(int(word[0]+word[2]+word[1]))

a = -10
print(split(str(a)))
x = True
while x == True:
    if(int(switch(split(str(a))) - 12 == a * 2)):
        print(a)
        x = False
    else:
        a = a - 1