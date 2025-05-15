# Solving radar detection problem with ASTAR pathfinding algorithm

> [!NOTE]
>This proyect aims to using the ASTAR pathfinding algorithm to find suitable routes for an anti-detection (stealth) plane going through an 
> 

## Set up: 
Once downloaded the code from the github [repo](https://github.com/Albrtito/UC3M-AI-RADARS) (the repo will be set public passed the due date) the python code may be run using python3 and with the requirements found on `src/py/requirements.txt`. To easily install all of them and create the python virtual enviroment the following commands can be run in the `src/py` folder: 

```bash
python3 -m venv .venv
pip install -r requirements.txt
```

Once the virtual enviroment has been set the whole project is good to run. Just go into the `py/` folder and run the main.py script (or if using an IDE, just runthe main.py script).

The script will take two arguments, the scenario and the plane tolerance. 
* The scenario sets the map, the positions of the radars and the points of interest the plane should go to POI
* Based on the tolerance set for the plane, it'll or wont be able to traverse certain parts of the maps. 


```bash
python3 main.py <scenario> <tolerance>
```

**Where**
* The scenario should have the format `scenario_<number>` with <number> being an integer from 0 to 10
* The tolerance should be a float that uses a point `.` as decimal separator

## Scripts and references: 
The github repo, not de delivered code, contains a folder with scripts used for creation of the zip file and a referencee folder with the problem statement. Any other scripts or refrendes used during this proyect will be added there. 


