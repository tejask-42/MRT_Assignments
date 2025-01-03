import datetime

data = []  # vehicle data
customer_data = []  # customer data
bookings = []  # bookings data


class Vehicle:
    def __init__(self, vehicle_id: str, make: str, model: str, year: int, rental_rate: float, availability: bool):
        self.vehicle_id = vehicle_id
        self.make = make # eg Toyota
        self.model = model # eg Camry
        if year < 1885:  # the first car was created in 1885
            raise ValueError("Invalid year of manufacture.")
        self.year = year
        if rental_rate <= 0:
            raise ValueError("Invalid rental rate.")
        self.rental_rate = rental_rate
        self.availability = availability
        self.user = None  # to integrate customer class later

    # apply some conditions on make and model to find out vehicle type
    vehicle_type = "car"

    def print_details(self):
        print(f'{self.vehicle_type} Brand: {self.make.lower().capitalize()}\n{self.vehicle_type} Model: {self.model}')
        print(f'Vehicle ID: {self.vehicle_id}\nYear of manufacturing: {self.year}')
        if self.availability:
            print(f"This {self.vehicle_type} is available for rental.")
            print(f'The rental price of this {self.vehicle_id} is: ${self.rental_rate}')
        else:  # if car not available for rental, no need to display rental rate
            print(f"This {self.vehicle_type} is currently not available for rent.")


# Example vehicle data
vehicle1 = Vehicle(vehicle_id="V001", make="Toyota", model="Camry", year=2020, rental_rate=50.0, availability=True)
vehicle2 = Vehicle(vehicle_id="V002", make="Tesla", model="Model S", year=2022, rental_rate=120.0, availability=True)
vehicle3 = Vehicle(vehicle_id="V003", make="Ford", model="Mustang", year=2018, rental_rate=85.0, availability=True)
vehicle6 = Vehicle(vehicle_id="V006", make="Ford", model="Mustang", year=2017, rental_rate=95.0, availability=True)
vehicle4 = Vehicle(vehicle_id="V004", make="Honda", model="Civic", year=2019, rental_rate=40.0, availability=True)
vehicle5 = Vehicle(vehicle_id="V005", make="Chevrolet", model="Tahoe", year=2021, rental_rate=95.0, availability=True)
data.append(vehicle1)
data.append(vehicle2)
data.append(vehicle3)
data.append(vehicle4)
data.append(vehicle5)
data.append(vehicle6)


def return_vehicle(vehicle_id: str, data: list):
    flag = False
    for vehicle in data:
        if vehicle.vehicle_id == vehicle_id:
            vehicle.availability = False
            print(f"{vehicle.make} {vehicle.model} returned successfully.")
            vehicle.user = None  # No one is using the car after customer returns it
            flag = True # checks if matching vehicle is found
            return
    if not flag:
        print("No such vehicle found in current database.")


def rent_vehicle(customer_id: str, rental_duration: int, make: str, model: str,
                 data: list,
                 customer_data: list):  #rental duration should be integer number of days (consider days as minutes)
    for customer in customer_data:
        if customer.customer_id == customer_id:
            if isinstance(customer, PremiumCustomer) and customer.discount:
                rental_duration *= 0.9  # changing rental duration to 0.9 times instead of changing rental rate as it will finally have same effect on price
            # if customer is regular customer, check loyalty points and apply similar discount based on company conditions
    flag = False
    for vehicle in data:
        if vehicle.make == make and vehicle.model == model and vehicle.availability:
            print(f"A {vehicle.make} {vehicle.model} is available for rent.")
            print(f"Price of renting for {rental_duration} days is ${rental_duration * vehicle.rental_rate}.")
            vehicle.availability = False  # assuming customer can pay the rent
            for customer in customer_data:
                if customer.customer_id == customer_id:
                    x = datetime.datetime.now()
                    for booking in bookings: # checking if some pre booking is clashing with this rental
                        if booking.vehicle.vehicle_id == vehicle.vehicle_id and not booking.completed and (x + datetime.timedelta(minutes=rental_duration) > booking.req_time):
                            print(f"This {vehicle.vehicle_type} is already booked by someone during this time period.")
                            return
                    if isinstance(customer, PremiumCustomer):
                        print("Thank you for being a premium customer!!")  # saying thanks when necessary
                        customer.rental_history.append(
                            [vehicle, x, rental_duration / 0.9])  # reverting rental duration to original for logging
                    else:
                        customer.rental_history.append( # no change if regular customer
                            [vehicle, x, rental_duration])
                    vehicle.user = customer
                    return  # assuming such a customer exists in customer database, so not checking for doesn't exist condition
            flag = True
            return
    if not flag: # using flag for checking no errors encountered
        print("No such vehicle found in database.")


class LuxuryVehicle(Vehicle):
    extra_features = ["Leather seats", "Inbuilt GPS", "Automatic gear", "Seat warmers"]

    def __init__(self, leather_seats=False, gps=False, auto=False, warmer=False, *args):
        Vehicle.__init__(self, *args)
        self.leather_seats = leather_seats
        self.gps = gps
        self.auto = auto
        self.warmer = warmer
        self.rental_rate *= 1.2

    def print_details(self):
        Vehicle.print_details(self)
        print(f"This {self.vehicle_type} also comes with additional features like: ")
        if self.leather_seats:  # one of these must be present in the vehicle for it to be considered a luxury vehicle so not checking the condition if none present
            print("Leather seats, ", end="")
        if self.gps:
            print("Inbuilt GPS, ", end="")
        if self.auto:
            print("Automatic gear, ", end="")
        if self.warmer:
            print("Seat warmers, ", end="")
        print()


class Customer:
    def __init__(self, customer_id: str, name: str, contact_info: str, rental_history: list):
        self.customer_id = customer_id
        self.name = name
        self.contact_info = contact_info
        self.rental_history = rental_history
        self.reminded = False  # to take care of remind function later

    def print_details(self):
        print(f"Customer Name: {self.name}")
        print(f"Customer Contact Information: {self.contact_info}")
        print(f"Customer ID: {self.customer_id}")

    def view_history(self):
        if self.rental_history:
            print(f"The rental history of {self.name} is: ")
            for vehicle, start, time in self.rental_history:  # assuming rental history provides data of time of rent and duration of rent also
                print(f'{vehicle.vehicle_type} Brand: {vehicle.make.lower().capitalize()})'
                      f'{vehicle.vehicle_type} Model: {vehicle.model}')
                print(f'Vehicle ID: {vehicle.vehicle_id}\nYear of manufacturing: {vehicle.year}')
                print(f"This {vehicle.vehicle_type} was rented at {start.time()} for {time} days.")

        else:
            print(f"{self.name} has not rented any vehicles yet.")


customer1 = Customer(customer_id="C001", name="Alice Johnson", contact_info="alice.j@example.com", rental_history=[])
customer2 = Customer(customer_id="C002", name="Bob Smith", contact_info="bob.s@example.com", rental_history=[])
customer3 = Customer(customer_id="C003", name="Charlie Brown", contact_info="charlie.b@example.com", rental_history=[])
customer_data.append(customer1)
customer_data.append(customer2)
customer_data.append(customer3)


class PremiumCustomer(
    Customer):  # maybe add time data to customer class - time when customer was created, then make premium after set amount of time
    def __init__(self, *args):
        Customer.__init__(self, *args)
        self.discount = True

    def apply_discount(self, wants_discount=True):
        self.discount = wants_discount  # maybe if customer does not want discount, taking care of that
        # this function will only be called if customer specifically does not want discount


class RegularCustomer(Customer): # initialise customer as regular customer and add function for using loyalty points for discount
    def __init__(self, *args):
        Customer.__init__(self, *args)
        self.loyalty_points = 0


class RentalManager:
    def __init__(self, vehicles: list):
        self.vehicles = vehicles

    def add_vehicle(self, vehicle: Vehicle):
        self.vehicles.append(vehicle)

    # instead of removing vehicles that are not available, keep them in list, marked as unavailable

    def report(self):
        for vehicle1 in self.vehicles:
            print("The report for all vehicles is as follows:")
            vehicle1.print_details()
            if vehicle1.user:  # if vehicle is currently rented
                print(f"This {vehicle1.vehicle_type} is currently rented by {vehicle1.user.name}.")
                print("Customer details:")
                vehicle1.user.print_details()
                for v, start, time in vehicle1.user.rental_history:
                    if v.vehicle_id == vehicle1.vehicle_id:
                        print(
                            f"{vehicle1.user.name} had rented this {vehicle1.vehicle_type} at {start.time()} for {time} days.")
                        break
                # check the rental history of the user of this vehicle, find vehicle1 in there


def remind(customer_data: list):  # remind customer one min(day) before their rent is due
    for customer in customer_data:
        if not customer.reminded:
            for v, start, time in customer.rental_history:  # can have multiple rented cars, give reminder for all
                if not v.availability:
                    due = start + datetime.timedelta(minutes=time)
                    x = datetime.datetime.now()
                    if (due - x) < datetime.timedelta(
                            minutes=1):  # if time remaining to due date is less than one min(day) then remind and set reminded to true
                        if isinstance(customer, PremiumCustomer):
                            print(
                                f"Hey, {customer.name} your rent of ${v.rental_rate * time * 0.9} is due for a {v.make} {v.model}.")  # discount for premium
                        else:
                            print(
                                f"Hey, {customer.name} your rent of ${v.rental_rate * time} is due for a {v.make} {v.model}.")
                        customer.reminded = True


# keep track of bookings and remove when completed
class Booking:
    def __init__(self, customer: Customer, vehicle: Vehicle, req_time, duration: int, completed=False):
        self.customer = customer
        self.vehicle = vehicle
        self.req_time = req_time
        self.duration = duration
        self.completed = completed


def book(customer_id: str, make: str, model: str, req_time, duration: int, bookings: list):
    flag = False
    for customer in customer_data:
        if customer.customer_id == customer_id:
            flag1 = False
            for vehicle in data:
                if vehicle.make == make and vehicle.model == model:
                    for booking in bookings:
                        start = datetime.datetime.now() + datetime.timedelta(minutes=duration)
                        if booking.vehicle.vehicle_id == vehicle.vehicle_id and booking.req_time <= start <= booking.req_time + datetime.timedelta(
                                minutes=duration):
                            print(f"This {vehicle.vehicle_type} is already booked.")
                            return  # check if same vehicle has been booked by someone in same interval of time as wanted by customer
                    booking1 = Booking(customer, vehicle, req_time, duration)
                    bookings.append(booking1)
                    print(f"Your {vehicle.vehicle_type} has been booked successfully for {duration} days.")
                    flag1 = True
                    flag = True
            if not flag1:
                print(f"No such {make} {model} found in database.")
                return
    if not flag:
        print(f"No such customer found in database.")


def givebooking(bookings: list, data: list,
                customer_data: list):  # actually gives vehicle to customer when booking time arrives
    x = datetime.datetime.now()
    for booking in bookings:
        if not booking.completed:
            if booking.req_time < x:  # if start time of booking already passed
                print(f"Initialising rental for booking of {booking.customer.name}.")
                rent_vehicle(booking.customer.customer_id, booking.duration, booking.vehicle.make, booking.vehicle.model, data,
                             customer_data)
                print(
                    f"A {booking.vehicle.make} {booking.vehicle.model} has been rented to {booking.customer.name} for {booking.duration} days.")
                booking.completed = True



def search(vehicles: list, criteria: list, sort_by="rental_ rate"):
    found = []
    if criteria[0] == "vehicle_type":
        vtype = input("Enter vehicle type: ")
        for vehicle in vehicles:
            if vehicle.vehicle_type == vtype:
                found.append(vehicle)
    elif criteria[0] == "make":
        vmake = input("Enter make: ")
        for vehicle in vehicles:
            if vehicle.make == vmake:
                found.append(vehicle)
    elif criteria[0] == "model":
        vmodel = input("Enter model: ")
        for vehicle in vehicles:
            if vehicle.model == vmodel:
                found.append(vehicle)
    elif criteria[0] == "year":
        vyear = input("Enter year: ")
        for vehicle in vehicles:
            if vehicle.year == vyear:
                found.append(vehicle)
    elif criteria[0] == "rental_rate":
        mid = float(input("Enter rate: "))
        vrange = float(input("Enter range of rate: "))
        for vehicle in vehicles:
            if mid - vrange <= vehicle.rental_rate <= mid + vrange:
                found.append(vehicle)
    elif criteria[0] == "availability":
        for vehicle in vehicles:
            if vehicle.availability:
                found.append(vehicle)
    else:
        print("Not a valid search criteria")
        return []
    if not found:
        print("No such vehicles found")
        return []
    criteria.pop(0)
    if criteria:
        return search(found, criteria, sort_by) # handles the case of multiple criteria for filtering
    if sort_by == "rental_rate":
        found.sort(key=lambda v: v.rental_rate)
    elif sort_by == "vehicle_id":
        found.sort(key=lambda v: v.vehicle_id)
    elif sort_by == "year":
        found.sort(key=lambda v: v.year)
    elif sort_by == "make":
        found.sort(key=lambda v: v.make)
    elif sort_by == "model":
        found.sort(key=lambda v: v.model)
    elif sort_by == "availability":
        found.sort(key=lambda v: v.availability)
    else:
        print("Not a valid sort criteria")
        return []
    return found


if __name__ == "__main__":
    print("Enter customer details.")
    cid = input("Enter customer id: ")
    cond = True
    while cond:  # have coded all functions but major issue in time control - print statement of booking done and reminder wont execute till next input is given
        # but next input is also needed if for example customer wants to rent two cars in succession
        remind(customer_data)
        givebooking(bookings, data, customer_data)
        flag2 = False
        for customer in customer_data:  # customer data is a list of customers
            if cid == customer.customer_id:
                flag2 = True
                print("Welcome back!")
                print("Do you want to rent(1), return(2), book(3) or search(4) a vehicle?, Quit(5)")
                command = int(input())
                if command == 1:
                    flag = False
                    for vehicle in data:  # data is a list of current vehicles - both available and unavailable
                        if vehicle.availability:
                            vehicle.print_details()
                            flag = True
                    if not flag:
                        print("No vehicles currently available to rent.")
                        break
                    else:
                        print("Which vehicle do you want to rent? Enter vehicle id")
                        vid = input()
                        flag1 = False
                        for vehicle in data:
                            if vid == vehicle.vehicle_id:
                                flag1 = True
                                duration = int(
                                    input(f"Enter the number of days you want to rent this {vehicle.vehicle_type}."))
                                rent_vehicle(cid, duration, vehicle.make, vehicle.model, data,
                                             customer_data)  # not checking for invalid inputs here
                elif command == 2:
                    vid = input("Enter the vehicle id of the car you want to return.")
                    flag = False
                    for vehicle in data:
                        if vid == vehicle.vehicle_id and not vehicle.availability:
                            flag = True
                            if vehicle in [v for v, s, t in customer.rental_history]:
                                return_vehicle(vid, data)
                            else:
                                print(f"No such {vehicle.vehicle_type} found to return.") # if customer tries to return a vehicle they dont have
                    if not flag:
                        print("No such unreturned vehicle found.")
                elif command == 3:
                    make = input("Enter the make of the vehicle you want to book: ")
                    model = input("Enter the model of the vehicle you want to book: ")
                    days = int(input("How many days from now do you want to book the vehicle?: "))
                    x = datetime.datetime.now()
                    duration = int(input("How many days do you want to book the vehicle?: "))
                    book(customer.customer_id, make, model, x + datetime.timedelta(minutes=days), duration, bookings)
                elif command == 4:
                    print("Available filters are: vehicle_type, make, model, year, rental_rate, availability")
                    n = int(input("How many filters do you want to apply?: "))
                    criteria = []
                    for i in range(n):
                        criteria.append(input(f"Enter filter {i + 1}: "))
                    print("You can sort according to rental_rate, vehicle_id, year, make, model, availability.")
                    sort_by = input("How do you want to sort the results?: ")
                    filtered = search(data, criteria, sort_by)
                    print("The filter returned: ")
                    for v in filtered:
                        v.print_details()
                elif command == 5:
                    print("Bye.")
                    cond = False
                else:
                    print("Not a valid command.")
        print() # for better readability
        print("-"*10)
        print()
        c = input(f"Do you want to change user(Y/N)? ").lower()
        if c == "y":
            cid = input("Enter new user customer id: ")
        if not flag2:
            print("No such existing customer found. Enter details to register.")
            # by default make regular customer, apply conditions to check if eligible for premium customer
            name = input("Enter your name: ")
            info = input("Enter you contact information: ")
            rhist = []
            newcust = RegularCustomer(cid, name, info, rhist)
            customer_data.append(newcust)
            print("Successfully registered new customer.")
        print()
        print("-"*10)
        print()
