#include <iostream>
#include <random>
#include <cstdlib>
#include <ctime>

using namespace std;

bool isValidInput(int guess) {
	return guess >= 0 && guess <= 100;
}
int Random_Number_Generator()
{
	int seed = time(0);
	srand(seed);
	int N = rand() % 101;
	return N;
}

void Guessing_Game(int guess)
{
	int attempts = 0;
	int attempts_limit = 6;
	int number = Random_Number_Generator();
	while (guess != number && attempts != attempts_limit)
	{
		bool check = isValidInput(guess);
		if (!check)
		{
			cerr << "Invalid input. Please enter a number between 0 and 100." << endl;
			attempts++;
		}
		if (guess < number && check)
		{
			cout << "Too low! Try again: " << endl;
			attempts++;
		}
		else if (guess > number && check)
		{
			cout << "Too high! Try again: " << endl;
			attempts++;
		}
		cin >> guess;
		if (guess == number)
		{
			cout << "Congratulations! You guessed the number!" << endl;
			cout << "Number of Invalid attempts are: " << attempts << endl;
			break;
		}
		if (attempts == attempts_limit)
		{
			cout << "You have reached the maximum number of attempts. The correct number was: " << number << endl;
		}
	}
}


int main()
{
	cout << "Guessing Game:........." << " " << endl;
	int guess;
	cout << "Enter your guess: " << endl;
	cin >> guess;
	Guessing_Game(guess);
	
	return 0;
}
