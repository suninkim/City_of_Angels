# test_gui.py

import tkinter as tk

def main():
    root = tk.Tk()
    label = tk.Label(root, text="Hello GUI from Raspberry Pi!")
    label.pack(padx=20, pady=20)
    root.mainloop()

if __name__ == "__main__":
    main()
