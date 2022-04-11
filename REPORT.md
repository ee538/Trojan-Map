# TROJAN MAP | PROJECT REPORT

Team Members : Sudharshan Subramaniam Janakiraman (USC ID : 8125560141), Amrith Coumaran (USC ID:)

# PROJECT FOCUS
This project focuses on using data structures in C++ and implementing various graph algorithms to build a map application. We will have small features similar to Google Map such as Auto Complete, Finding the exact location of a place in the map. This project will also involve analysis of time complexity of each functions utilized to full fill a particular application as well as tet case behavior analysis to check if the written code indeed works as expected.

# MAP FEATURES

- Autocomplete : Displays a list of location based on the partial case insensitive input by the user
- FindLocation : Finds the location (Latitude and Longitude) given an input location (Exactr Match) otherwise finds the location of the lexicographically closest matching location to the word

## Utility Functions
- ```double GetLat(const std::string& id);```  returns the latitude given the unique id : Time Complexity  O(1)
- ```double GetLon(const std::string& id);``` : returns the longitude given the unique id : TIme Complexity O(1)
- ```std::string GetName(const std::string& id);``` : returns the Name of the location given the unique id: Time COmplexity O(1)
- ```std::string GetID(const std::string& name);``` : Returns the Unque ID if the given name exists in the database :  Time Complexity O(n) where n = number of unique id in the database
- ```std::vector<std::string> GetNeighborIDs(const std::string& id);``` : Returns the list of neighbour id's given the unique id: Time Complexity O(1).

These Functions will be called by other functions in the project to access the required data which is stored in a map <Unique id, Node>.

### Feature 1 : Auto Complete

### Feature 2 : Find The Location

This Feature focuses on finding the location in the map (Latitude and Longitude) based on the input provided by the user. The input is of type std::string. 

The Declation of the function is given as ```std::pair<double, double> GetPosition(std::string name);```

Flow Chart of the Function:
<p align="center"><img src="img/Feature21.png" alt="Trojan" width="500" /></p>

As We can see that there is one loop which will help us finding the location given  the name, 

The ```Time Complexity = O(n)``` where n = # Unique ID's in the data

Observation : This GetPosition Will only return proper location if the user inputs correct case sensitive data, Otherwise it will return (-1, -1). This provides a limitation when the user doesnt follow case sensitivity or Makes spelling mistakes in Input

In order the rectify the above limitation we will make use of two more function CalculateEditDistance and FindClosestName. as the name suggests, The FindClosestName function will find if there are any names which are lexicographically closer to the user's input name. For Finding such name, this function will make use of CalculateEditDIstance which returns the EditDistance between two string ( i.e., Number of operations needed (Insert delete, replace) to convert one string to another string). Find Closest Name will run the input name against all names in the data and will select the name from data which has minimum EditDistance.

The declaration of the two function in the program are given as 

Declaration of CalculateEditDistance Function : ```int CalculateEditDistance(std::string, std::string);```

This Function Takes 2 strings as input and returns the edit distance between them

Flow Chart of CalculateEditDistance FUnction:
<p align="center"><img src="img/CalculateEditDistance.png" alt="Trojan" width="500" /></p>

The ```Time Complexity = O(mn)``` where m = Length of String 1, n = Length of String 2

Declaration of FindClosestName Function : ```std::string FindClosestName(std::string name);```

This Function takes an input name and returns a name(string) which is lexicographically closer to the input name (string)

Flow Chart of FindClosestName FUnction:
<p align="center"><img src="img/FindClosestName.png" alt="Trojan" width="500" /></p>

The ```Time Complexity = O(nlp)``` where n = # unique ID's, l = Length of Input Name, p = Length of the Largest Name in the data 

RESULTS :
<p align="center"><img src="img/GetPosOutput1.png" alt="Trojan" width="500" /></p>
<p align="center"><img src="img/GetPosMap1.png" alt="Trojan" width="500" /></p>

