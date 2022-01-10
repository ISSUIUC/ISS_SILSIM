import os
import pyautogui
import time
from pathlib import Path

def main():

    out_pth = os.getcwd()[:-3] + "output\\"
    ras_pth = os.getcwd()[:-3] + "cdx1\\"

    pyautogui.PAUSE = 0.6
    pyautogui.FAILSAFE = True

    protuberance = 100
    alpha = 15
    nozzle_exit_diameter = 2.7

    ned_string = str(nozzle_exit_diameter).zfill(5)

    rasaero_location = "C:\ProgramData\Microsoft\Windows\Start Menu\Programs\RASAero II\RASAero II"
    rasaero_file = "Test_5800_mk3_{}.cdx1".format(str(protuberance).zfill(3))

    os.startfile(rasaero_location)
    screenWidth, screenHeight = pyautogui.size()

    pyautogui.moveTo(screenWidth * 0.5, screenHeight * 0.5)
    pyautogui.click()

    pyautogui.press('alt')
    pyautogui.press('enter')

    pyautogui.hotkey('ctrl', 'o')
    pyautogui.hotkey('ctrl', 'l')
    pyautogui.press('delete')
    pyautogui.write(ras_pth)
    pyautogui.press('enter')
    for i in range(6):
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

    for i in range(alpha):
        
        output_path = (out_pth + "{}_{}.txt").format(str(i).zfill(2), str(protuberance).zfill(3))

        pyautogui.hotkey('ctrl', 'a')
        pyautogui.write(output_path)
        for j in range(4):
            pyautogui.press('tab')
        pyautogui.hotkey('ctrl', 'a')
        pyautogui.write(str(i))

        for j in range(3):
            pyautogui.press('tab')
        pyautogui.press('enter')

        for j in range(2):
            pyautogui.hotkey('shift', 'tab')

    for i in range(2):
            pyautogui.press('tab')
    pyautogui.press('enter')

if __name__ == "__main__":
   main()






