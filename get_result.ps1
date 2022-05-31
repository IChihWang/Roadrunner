rm result/*


For ($delay=0; $delay -le 50; $delay+=5) {
  For ($load=0.05; $load -le 0.65; $load+=0.05) {
      python .\main.py $load 4 0 T $delay
  }
}
