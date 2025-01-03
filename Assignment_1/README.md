Vehicle Rental Management System Project Report

Overview

This Python code implements a simple vehicle rental management system. It allows customers to rent, return, book, and search for vehicles. The system also handles customer registration, booking requests, and reminder functionality for vehicle rentals. Additionally, the system manages different types of vehicles, including luxury and regular vehicles, and supports both regular and premium customers.



Problems Faced

While working on the booking function, it was necessary to track time so that the user would get notified that it is time to receive their vehicle. Time tracking would also need to be used to check if two bookings of the same vehicle are clashing or to make sure another user does not rent a car that is booked later during this rent time. To add this functionality, I used the datetime module, first converting the time span of a day in reality to a minute in the program—for quicker testing.

The givebooking() and remind() functions are continuously called in the main loop, but I ran into one major problem with this approach. I planned to run a while loop that continually takes user input for renting, booking, returning, and searching vehicles. This was to take care of the case where the user might want to rent multiple vehicles at a time. But if the CLI is asking for some input and the user delays in giving input, the remind function will not be called until the user provides the input, thus delaying the reminder function. To solve this, one option is to implement threading and Queues, which will allow the time-related functions to run in parallel with the main program without interfering with the inputs.
