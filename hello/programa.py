vocales = ['A','E','I','O','U']
space = [' ']
vocal3s = 0
consonantes = 0
espacios = 0

string = input("Ingresar: ").upper()
for str in string:
    if str in vocales:
        vocal3s += 1
    elif str in space:
        espacios += 1
    else:
        consonantes += 1

print("Vocales: ", vocal3s)
print("Consonantes: ", consonantes)
