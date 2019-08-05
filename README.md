# nml2_goc
 Learning NeuroML2 with cerebellar Golgi modelling

## Biophysical Mechanisms

### Ion Channels
- Based on mod files from [Solinas et al, 2007](https://github.com/OpenSourceBrain/SolinasEtAl-GolgiCell/). Original publication: Solinas S, Forti L, Cesana E, Mapelli J, De Schutter E, D’Angelo E. Computational reconstruction of pacemaking and intrinsic electroresponsiveness in cerebellar Golgi cells. [Front Cell Neurosci. 2007;1:2](http://www.ncbi.nlm.nih.gov/pubmed/18946520)
- NeuroML2 implementation verified against OSB version of [Vervaeke et al. 2010](https://www.zenodo.org/badge/latestdoi/4960822). Original publication: Rapid Desynchronization of an Electrically Coupled Interneuron Network with Sparse Excitatory Synaptic Input, [Neuron 2010](http://www.sciencedirect.com/science/article/pii/S089662731000512X).

### Morphology
- Currently has reduced morphology in [test_channel.cell.nml](https://github.com/harshagurnani/nml2_goc/blob/master/Mechanisms/test_channel.cell.nml)

*NOTE: Single compartment doesn't have right geometry/membrane res and causes unrealistic currents - could be optimized in future* 

### Golgi Cell
- Single cell with 2 Ca pools: [NeuroML2 file](https://github.com/harshagurnani/nml2_goc/blob/master/Mechanisms/test_channel.cell.nml)
- Single cell with one Ca pool (all channels read/write onto same pool): [NeuroML2 mechanisms and cell](https://github.com/harshagurnani/nml2_goc/tree/master/Mechanisms/onePool)

## Running simulations
Using NEURON for simulation (via pyneuroml)


- From NML2 descriptions:

```python
python run_simple_goc.py   #
```
Generates LEMS simulation file (LEMS_sim1_goc.xml), NML->mod file, hoc file, NEURON-python simulation file, compiles Mod files and runs the Nrn-python simulation.

- If nrn-python is already created/ edited externally, run:
```python
python LEMS_sim_goc1_nrn.py 
```

### Available sims:
- simple_cell with HCN and leak channels : 
```GoC_file_name = 'simple_cell.cell.nml'```
- single compartment GoC with all channels from Solinas et al 2007: (!!!!Currently failing because of some channels)
```GoC_file_name = 'test_channel.cell.nml'```
