##
#  @file        script.py
#  @authors     Rishi Gottumukkala
# 
#  @brief       Generation of RASAero simulation files
#

import os
import pyautogui
import time
from pathlib import Path

def main():

    # define paths
    out_pth = os.getcwd()[:-3] + "output\\"
    ras_pth = os.getcwd()[:-3] + "cdx1\\"

    # define PyAutoGUI interface
    pyautogui.PAUSE = 0.8
    pyautogui.FAILSAFE = True

    # define simulation parameters
    protuberances = []
    alpha = 16
    nozzle_exit_diameter = 2.7
    ned_string = str(nozzle_exit_diameter).zfill(5)

    # RASAero application 
    rasaero_location = "C:\ProgramData\Microsoft\Windows\Start Menu\Programs\RASAero II\RASAero II"
    print('Executing program at C:\ProgramData\Microsoft\Windows\Start Menu\Programs\RASAero II\RASAero II')
    os.startfile(rasaero_location)
    screenWidth, screenHeight = pyautogui.size()
 
    # fetch protuberance files
    files = os.listdir(ras_pth)
    for i in range(len(files)):
        filename_length = len(files[i])
        protuberances.append(int(files[i][filename_length-8:filename_length-5]))

    # run sims for each protuberance
    for i in range(len(protuberances)):

        # define parameters to open cdx1 file
        protuberance = protuberances[i]
        rasaero_file = "Intrepid_5800_mk6_{}.cdx1".format(str(protuberance).zfill(3))
        pyautogui.moveTo(screenWidth * 0.5, screenHeight * 0.5)
        pyautogui.click()
        pyautogui.press('alt')
        pyautogui.press('enter')
        pyautogui.hotkey('ctrl', 'o')
        pyautogui.PAUSE = 1.5
        
        # accounting for non-initial protuberance interations
        if (i != 0):
            pyautogui.press('right')
            pyautogui.press('enter')

        # open file directory
        pyautogui.hotkey('ctrl', 'l')
        pyautogui.press('delete')
        pyautogui.write(ras_pth)
        pyautogui.press('enter')
        pyautogui.PAUSE = 1.5
        
        # open file
        for j in range(6):
                pyautogui.press('tab')
        pyautogui.write(rasaero_file)
        pyautogui.press('enter')

        # define nozzle exit diameter
        pyautogui.PAUSE = 0.01
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

        # iterate through each angle of attack
        for j in range(alpha):

            print(f"Running Alpha = {j}")
            pyautogui.PAUSE = 0.1
        
            # define output file name
            output_path = (out_pth + "{}_{}.txt").format(str(j).zfill(2), str(protuberance).zfill(3))

            # input angle of attack for iteration
            pyautogui.hotkey('ctrl', 'a')
            pyautogui.write(output_path)
            for k in range(4):
                pyautogui.press('tab')
            pyautogui.hotkey('ctrl', 'a')
            pyautogui.write(str(j))

            # run sim
            for k in range(3):
                pyautogui.press('tab')
            pyautogui.press('enter')

            # reset for next angle of attack sim
            for k in range(2):
                pyautogui.hotkey('shift', 'tab')

        # reset for next file
        for j in range(2):
                pyautogui.press('tab')
        pyautogui.press('enter')

if __name__ == "__main__":
   main()






