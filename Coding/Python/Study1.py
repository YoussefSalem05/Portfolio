def main():
    grades=None
    while grades is None:
        try:
            temp=input("Enter list of grades separated by commas: ")
            print("You entered:", temp)
            temp=temp.split(",")
            temp=[int(g) for g in temp]
            if any(g<0 or g>100 for g in temp):
                print("Grades must be between 0 and 100.")
                continue
            grades=temp
        except ValueError:
            print("Invalid range.")
            grades=None

    if len(grades)!=0:    
     average=sum(grades)/len(grades) 
     print("Average grade:", average)
     if average>=90:
            print("Status:Excellent")
    elif average<=89 and average>=75:
            print("Status:Good")
    else:
            print("Status:Needs Improvement")
        
if __name__ == '__main__':
    main()
