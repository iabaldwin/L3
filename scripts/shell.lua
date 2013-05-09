local f = io.popen("ls -lah /Users/ian/code/datasets/" )

if f then
    print(f:read("*a"))
else
    print("failed to read")
end
