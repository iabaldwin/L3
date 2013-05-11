require'lfs'
--for file in lfs.dir[[C:\Program Files]] do
for file in lfs.dir[[/Users/ian/code/datasets/]] do
    if lfs.attributes(file,"mode") == "file" then print("found file, "..file)
    elseif lfs.attributes(file,"mode")== "directory" then print("found dir, "..file," containing:")
        --for l in lfs.dir("C:\\Program Files\\"..file) do
        for l in lfs.dir("/Users/ian/code/datasets/"..file) do
            print("",l)
        end
    end
end