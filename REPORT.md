# TROJAN MAP | PROJECT REPORT

Team Members : Sudharshan Subramaniam Janakiraman (USC ID : 8125560141), Amrith Coumaran (USC ID:)

# PROJECT FOCUS
This project focuses on using data structures in C++ and implementing various graph algorithms to build a map application. We will have small features similar to Google Map such as Auto Complete, Finding the exact location of a place in the map. This project will also involve analysis of time complexity of each functions utilized to full fill a particular application as well as tet case behavior analysis to check if the written code indeed works as expected.

# MAP FEATURES

- Autocomplete : Displays a list of location based on the partial case insensitive input by the user
- FindLocation : Finds the location (Latitude and Longitude) given an input location (Exactr Match) otherwise finds the location of the lexicographically closest matching location to the word

## Utility Functions
- double GetLat(const std::string& id); : returns the latitude given the unique id : Time Complexity  O(1)
- double GetLon(const std::string& id); : returns the longitude given the unique id : TIme Complexity O(1)
- std::string GetName(const std::string& id); : returns the Name of the location given the unique id: Time COmplexity O(1)
- std::string GetID(const std::string& name); : Returns the Unque ID if the given name exists in the database :  Time Complexity O(n) where n = number of unique id in the database
- std::vector<std::string> GetNeighborIDs(const std::string& id); : Returns the list of neighbour id's given the unique id: Time Complexity O(1).

These Functions will be called by other functions in the project to access the required data which is stored in a map <Unique id, Node>.

### Feature 1 : Auto Complete

### Feature 2 : Find The Location

This Feature focuses on finding the location in the map (Latitude and Longitude) based on the input provided by the user. The input is of type std::string. 

<p align="center"><img src="img/Feature21.png" alt="Trojan" width="500" /></p>

