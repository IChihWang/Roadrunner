
rm result\result.csv
For ($i=0.05; $i -le 1.1; $i+=0.05) {
	For ($seed=0; $seed -le 10; $seed++) {
		python .\main.py $i $seed 0 F
		python .\main.py $i $seed 1 F
		python .\main.py $i $seed 2 F
		python .\main.py $i $seed 0 T
		python .\main.py $i $seed 1 T
		python .\main.py $i $seed 2 T
	}
}