import os
import pyautogui
import time

def main():

    pyautogui.PAUSE = 0.4
    pyautogui.FAILSAFE = True

    protuberance = 0
    alpha = 15
    nozzle_exit_diameter = 2.7

    rasaero_location = "C:\ProgramData\Microsoft\Windows\Start Menu\Programs\RASAero II\RASAero II"
    rasaero_file = "{}_protub.CDX1".format(protuberance)

    os.startfile(rasaero_location)
    screenWidth, screenHeight = pyautogui.size()

    pyautogui.moveTo(screenWidth * 0.5, screenHeight * 0.5)
    pyautogui.click()

    pyautogui.press('alt')
    pyautogui.press('enter')

    pyautogui.hotkey('ctrl', 'o')

    pyautogui.write(rasaero_file)
    pyautogui.press('enter')

    pyautogui.press('alt')
    pyautogui.press('right')
    pyautogui.press('enter')
    pyautogui.press('down')
    pyautogui.press('enter')

    pyautogui.PAUSE = 0.001

    for i in range(alpha):

        output_path = "C:\\Users\\rishi\\Documents\\GitHub\\ISS_SILSIM\\utils\\RASAero_fetch\\output\\{}_{}.txt".format(str(i).zfill(2), str(protuberance).zfill(3))

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






