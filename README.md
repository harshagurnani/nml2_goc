# nml2_goc
 Learning NeuroML2 with cerebellar Golgi modelling

## Biophysical Mechanisms

### Ion Channels
- Based on mod files from [Solinas et al, 2007](http://www.ncbi.nlm.nih.gov/pubmed/18946520) - Solinas S, Forti L, Cesana E, Mapelli J, De Schutter E, D’Angelo E. Computational reconstruction of pacemaking and intrinsic electroresponsiveness in cerebellar Golgi cells. Front Cell Neurosci. 2007;1:2

## Running simulations
Using jneuroml for simulation (via pyneuroml)


- Creates LEMS simulation file (LEMS_sim1_goc.xml) and runs it:
```python
python run_simple_goc   #
```

- If .xml is created successfully, but run fails due to python packages, try the following after starting python:
```python
import neuroml as nml
from pyneuroml import pynml
from pyneuroml.lems.LEMSSimulation import LEMSSimulation
import lems.api as lems

#Runs and outputs xxx.v.dat in same directory
pynml.run_lems_with_jneuroml('LEMS_sim_goc1.xml', max_memory="1G", nogui=True, plot=False)  
```

### Available sims:
- simple_cell with HCN and leak channels : 
```GoC_file_name = 'simple_cell.cell.nml'```
- single compartment GoC with all channels from Solinas et al 2007: (!!!!Currently failing because of some channels)
```GoC_file_name = 'simple_cell.cell.nml'```
