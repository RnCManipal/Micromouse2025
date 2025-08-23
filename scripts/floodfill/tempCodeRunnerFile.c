printf("Wall data for current node: \n");
        for (int i =0 ; i<4; i++){
            int temp;
            wall_data[position[0]][position[1]][i] = dup_arr[position[0]][position[1]][i];
            printf("%d ", wall_data[position[0]][position[1]][i]);
        }