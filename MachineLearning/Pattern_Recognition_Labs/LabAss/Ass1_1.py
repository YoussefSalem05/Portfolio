import random
def guessing_game(max,*, attempts):
    number=random.randint(1, max)
    print("Welcome to the guessing game!")
    print(f"I'm thinking of a number between 1 and {max}.")
    guess=None
    guesses_tuple=()
    while len(guesses_tuple) < attempts:
            try:
                guess=int(input("Enter your guess: "))
                guesses_tuple=guesses_tuple+(guess,)
                if guess < number:
                    print("Too low! Try again.")
                    print(f"attempts left: {attempts - len(guesses_tuple)}")
                elif guess > number:
                    print("Too high! Try again.")
                    print(f"attempts left: {attempts - len(guesses_tuple)}")
                else:
                    print("Congratulations! You've guessed the number!")
                    print(f"Your guesses: {guesses_tuple}")
                    return(True, guesses_tuple, number)
            except ValueError:
                print("Invalid input. Please enter a valid number.")
    print("You ran out of attempts! The number was:", number)
    return(False, guesses_tuple, number)

def play_game() -> None:
    """
    Controls the game flow.
    - calls guessing_game
    - verifies the win/loss logic using assertions
    - handles replays
    """
    while True:
        # 1. Call the function with max=20 and attempts=5 (keyword arg)
        # We unpack the tuple result into three variables
        has_won, guesses, secret_number = guessing_game(20, attempts=5)

        # 2. Handle Win Condition
        if has_won:
            # Verify the winning number is actually in the history
            try:
                assert secret_number in guesses
            except AssertionError:
                print(f"Error: Logic Check Failed! You won, but {secret_number} is not in {guesses}.")

            print("Great job! Ending program.")
            break  # End the program

        # 3. Handle Loss Condition
        else:
            print(f"Game Over. The number was {secret_number}.")

            # Verify the winning number is NOT in the history
            try:
                assert secret_number not in guesses
            except AssertionError:
                print(f"Error: Logic Check Failed! You lost, but {secret_number} WAS in {guesses}.")

            # Ask to play again
            replay = input("Do you want to play again? (yes/no): ").lower()
            if replay == "yes":
                print("\nRestarting...")
                continue  # Go back to the top of the while loop
            else:
                print("Thanks for playing!")
                break  # Break the loop to end the program

# This line runs the game when you press play
if __name__ == "__main__":
    play_game()
