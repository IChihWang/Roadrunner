
rm result/*

For ($j=1000; $j -le 2000; $j+=1) {
	For ($i=0.6; $i -le 0.9; $i+=0.1) {
		python main.py $i $j 0 T
		python main.py $i $j 0 F
	}
}
