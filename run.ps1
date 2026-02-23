pio run -t clean -e xiao-esp32s3
pio run -t upload -e xiao-esp32s3; if ($LASTEXITCODE -eq 0) { pio device monitor -p COM4 }
