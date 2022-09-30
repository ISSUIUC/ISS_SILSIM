from PIL import Image
import os



'''
This algorithm is atrocious: It's slow, and it sometimes creates weird visual artifacts.
I just wrote this because I couldn't be bothered to manually smooth the b3ql1 data and I needed something that just kinda worked. 
'''

cwd = os.getcwd()
print(cwd)
path = cwd + './animation/python_scripts/input.png'
output_path = cwd + './animation/python_scripts/output.png'
texture = Image.open(path)
texture = texture.convert('L')



#threshold that needs to be passed to correct the pixel. multiplyed by 2.55 because paint.net measures grey values from 0-100, but grayscale values are saved 0-255
threshold = 2.55 * 6


#traverse pixels left to right, row by row. 
#if the difference of grayscale value of the current pixel and the previous pixel exceeds the threshold, replace the current pixel with the previous pixel
count = 0
for y in range(0, texture.height):
    for x in range(1, texture.width):
        if (abs(texture.getpixel((x, y)) - texture.getpixel((x - 1, y)))  >= threshold) :
            count += 1
            color = texture.getpixel((x - 1, y))
            texture.putpixel( (x, y), color)

#traverse pixels right to left, row by row. 
#if the difference of grayscale value of the current pixel and the previous pixel exceeds the threshold, replace the current pixel with the previous pixel
for y in range(0, texture.height):
    for x in range(texture.width - 2, 0, -1):
        if (abs(texture.getpixel((x, y)) - texture.getpixel((x + 1, y)))  >= threshold) :
            #print(f'Threshold passed; Original pixel: {texture.getpixel((x,y))}; New pixel: {texture.getpixel((x - 1,y))} ')
            
            count += 1
            color = texture.getpixel((x - 1, y))
            texture.putpixel( (x, y), color)

#traverse pixels up to down, column by column. 
#if the difference of grayscale value of the current pixel and the previous pixel exceeds the threshold, replace the current pixel with the previous pixel
for x in range(0, texture.width):
    for y in range(1, texture.height):
        if (abs(texture.getpixel((x, y)) - texture.getpixel((x, y - 1)))  >= threshold) :
            #print(f'Threshold passed; Original pixel: {texture.getpixel((x,y))}; New pixel: {texture.getpixel((x - 1,y))} ')
            
            count += 1
            color = texture.getpixel((x - 1, y))
            texture.putpixel( (x, y), color)



#traverse pixels down to up, column by column. 
#if the difference of grayscale value of the current pixel and the previous pixel exceeds the threshold, replace the current pixel with the previous pixel
for x in range(0, texture.width):
    for y in range(texture.height - 2, 0, -1):
        if (abs(texture.getpixel((x, y)) - texture.getpixel((x, y + 1)))  >= threshold) :
            #print(f'Threshold passed; Original pixel: {texture.getpixel((x,y))}; New pixel: {texture.getpixel((x - 1,y))} ')            
            count += 1
            color = texture.getpixel((x - 1, y))
            texture.putpixel( (x, y), color)

texture.save(output_path)
print(f'{count} pixels corrected')