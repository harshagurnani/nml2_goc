set root=C:\Users\Harsha\Anaconda3
call %root%\Scripts\activate.bat %root%
call conda activate nml2

FOR /L %%A IN (1,1,2) DO(
  python run_varied_GoC_single.py %%A
)
for %%f in (LEMS_sim_gocnet_GoC*.py) do (
    echo %%~nf
	python "%%~nf.py"
)