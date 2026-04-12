import tkinter as tk

def button_click(item):
    """Handles button clicks and keyboard inputs to update the display."""
    current = display_var.get()
    display_var.set(current + str(item))

def button_clear(event=None):
    """Clears the entire calculator display (Bound to Escape)."""
    display_var.set("")

def button_delete(event=None):
    """Deletes the last character typed (Bound to Backspace)."""
    current = display_var.get()
    display_var.set(current[:-1])

def button_equal(event=None):
    """Evaluates the math expression in the display (Bound to Enter/Return)."""
    try:
        # Swap the visual '²' for Python's actual '**2' math operator
        expression = display_var.get().replace('²', '**2')
        result = str(eval(expression))
        display_var.set(result)
    except ZeroDivisionError:
        display_var.set("Error: Div by 0")
    except Exception:
        display_var.set("Error")

def key_handler(event):
    """Listens to keyboard presses and triggers the corresponding functions."""
    # Allow standard numbers and operators to be typed
    valid_chars = "0123456789+-*/.%"
    if event.char in valid_chars:
        button_click(event.char)
    elif event.char == '=':
        button_equal()

# 1. Set up the main window
root = tk.Tk()
root.title("Calculator")
root.geometry("345x485")
root.resizable(0, 0)

display_var = tk.StringVar()

# 2. Create the display screen
display = tk.Entry(root, textvariable=display_var, font=('Arial', 20, 'bold'), 
                   bg="#f0f0f0", readonlybackground="#f0f0f0", bd=10, 
                   justify="right", state="readonly")
display.grid(row=0, column=0, columnspan=4, ipadx=8, ipady=20, pady=10)

# 3. Bind Keyboard Events
root.bind('<Key>', key_handler)               # Captures standard numbers and operators
root.bind('<Return>', button_equal)           # Enter key
root.bind('<KP_Enter>', button_equal)         # Numpad Enter key
root.bind('<BackSpace>', button_delete)       # Backspace key
root.bind('<Escape>', button_clear)           # Esc key

# 4. Define the buttons (Changed to the '²' symbol)
buttons = [
    ('C', 1, 0), ('DEL', 1, 1), ('%', 1, 2), ('/', 1, 3),
    ('7', 2, 0), ('8', 2, 1), ('9', 2, 2), ('*', 2, 3),
    ('4', 3, 0), ('5', 3, 1), ('6', 3, 2), ('-', 3, 3),
    ('1', 4, 0), ('2', 4, 1), ('3', 4, 2), ('+', 4, 3),
    ('0', 5, 0), ('.', 5, 1), ('²', 5, 2), ('=', 5, 3)
]

# 5. Place the buttons on the grid
for (text, row, col) in buttons:
    if text == '=':
        btn = tk.Button(root, text=text, font=('Arial', 16, 'bold'), fg="white", bg="#4caf50", 
                        command=button_equal, height=2, width=5)
    elif text == 'C':
        btn = tk.Button(root, text=text, font=('Arial', 16, 'bold'), fg="white", bg="#f43636", 
                        command=button_clear, height=2, width=5)
    elif text == 'DEL':
        btn = tk.Button(root, text=text, font=('Arial', 16, 'bold'), fg="white", bg="#ff9800", 
                        command=button_delete, height=2, width=5)
    else:
        btn = tk.Button(root, text=text, font=('Arial', 16, 'bold'), fg="black", bg="#e0e0e0",
                        command=lambda t=text: button_click(t), height=2, width=5)
    
    # takefocus=0 ensures the buttons don't trap the keyboard focus, keeping our hotkeys working
    btn.config(takefocus=0) 
    btn.grid(row=row, column=col, padx=5, pady=5)

# Run the application
root.mainloop()
