import os
import pyautogui
import time
from pathlib import Path

def main():

    out_pth = os.getcwd()[:-3] + "output\\"
    ras_pth = os.getcwd()[:-3] + "cdx1\\"

    pyautogui.PAUSE = 0.8
    pyautogui.FAILSAFE = True

    protuberances = []
    alpha = 16
    nozzle_exit_diameter = 2.7

    ned_string = str(nozzle_exit_diameter).zfill(5)

    rasaero_location = "C:\ProgramData\Microsoft\Windows\Start Menu\Programs\RASAero II\RASAero II"

    print('Executing program at C:\ProgramData\Microsoft\Windows\Start Menu\Programs\RASAero II\RASAero II')

 
    files = os.listdir(ras_pth)
    for i in range(len(files)):
        protuberances.append(int(files[i][14:17]))



    os.startfile(rasaero_location)
    screenWidth, screenHeight = pyautogui.size()

    for i in range(len(protuberances)):

        protuberance = protuberances[i]
        rasaero_file = "Test_5800_mk3_{}.cdx1".format(str(protuberance).zfill(3))

        pyautogui.moveTo(screenWidth * 0.5, screenHeight * 0.5)
        pyautogui.click()

        pyautogui.press('alt')
        pyautogui.press('enter')
        pyautogui.hotkey('ctrl', 'o')

        if (i != 0):
            pyautogui.press('right')
            pyautogui.press('enter')

        pyautogui.hotkey('ctrl', 'l')
        pyautogui.press('delete')
        pyautogui.write(ras_pth)
        pyautogui.press('enter')
        for j in range(6):
                pyautogui.press('tab')


        pyautogui.write(rasaero_file)
        pyautogui.press('enter')

        pyautogui.PAUSE = 0.001

        pyautogui.press('alt')
        pyautogui.press('right')
        pyautogui.press('enter')
        pyautogui.press('down')
        pyautogui.press('enter')

        pyautogui.hotkey('shift', 'tab')
        pyautogui.hotkey('ctrl', 'a')
        pyautogui.press('delete')
        pyautogui.write(ned_string)
        pyautogui.press('tab')

        for j in range(alpha):
        
            output_path = (out_pth + "{}_{}.txt").format(str(j).zfill(2), str(protuberance).zfill(3))

            pyautogui.hotkey('ctrl', 'a')
            pyautogui.write(output_path)
            for k in range(4):
                pyautogui.press('tab')
            pyautogui.hotkey('ctrl', 'a')
            pyautogui.write(str(j))

            for k in range(3):
                pyautogui.press('tab')
            pyautogui.press('enter')

            for k in range(2):
                pyautogui.hotkey('shift', 'tab')

        for j in range(2):
                pyautogui.press('tab')
        pyautogui.press('enter')

        pyautogui.PAUSE = 0.5

if __name__ == "__main__":
   main()






