
f = open("image.png", 'rb')
while(1):
    a = f.read(190)
    if len(a) <= 0 :
        break
    print(a)
        
f.close()



