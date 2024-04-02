using DataStructures
using Printf

function parse(filename::String)

	f = open(filename, "r")
	
	s::String = ""
	x::Int64 = 0
	y::Int64 = 0
	
	s = readline(f)
	if(s != "type octile")
		throw( error("fichier invalide" ) )
	end
	
	s = readline(f)
	y = Base.parse(Int64, s[8:end]) #on récupère la fin de la string ou il y a un int
	
	s = readline(f)
	x = Base.parse(Int64, s[7:end])
	
	s = readline(f)
	if(s != "map")
		throw( error("fichier invalide" ) )
	end
	
	map::Vector{String} = Vector{String}(undef, y)
	
	i::Int64 = 1
	while ! eof(f)
		s = readline(f)
		map[i] = s
		i += 1
	end
		
	close(f)
	
	return map
	
end

struct DistPos
#contient les données nécessaire pour selectionner le voisin le plus proche pour dijstra et A*
#type immutable
	pos::Tuple{Int64, Int64}
	
	pred::Tuple{Int64, Int64}
	#permet de reconstruire le chemin
	#	est (0,0) si c'est le point de départ (pas de predécesseur)
	
	init_empty() = new()
	DistPos(pos, pred) = new(pos, pred)
end

function estMur(map::Vector{String}, pos::Tuple{Int64, Int64})
	return map[pos[1]][pos[2]] == '@'
end

function buildPath(predessesorMap::Array{Tuple{Int64, Int64}}, f::Tuple{Int64, Int64} )

	path::Vector{Tuple{Int64, Int64}} = Vector{Tuple{Int64, Int64}}()
	current::Tuple{Int64, Int64} = f
	
	while (current != (0,0) )
		pushfirst!(path, current)
		current = predessesorMap[CartesianIndex(current)]
	end
	
	return path
end

function floodfill(map::Vector{String}, start::Tuple{Int64, Int64}, finish::Tuple{Int64, Int64})
#precondition : la carte ne doit pas être vide

	sizeX::Int64 = length(map)
	sizeY::Int64 = length(map[1])

	isSeen::Array{Bool} = fill(false, (sizeX, sizeY) )
	
	predessesorMap::Array{Tuple{Int64, Int64}} = Array{Tuple{Int64, Int64}}(undef, (sizeX, sizeY) )
	# permet de reconstruire le passage -> tableau des distances actuelles et des prédécesseurs
	
	heap::MutableBinaryHeap{Tuple{Int64, DistPos}} = MutableBinaryHeap{Tuple{Int64, DistPos}}( Base.By(first) )
	#tas Min/max qui permet de "trier" les valeurs
	
	is_finished::Bool = false
	
	current::DistPos = DistPos(start, (0,0) )
	# la position (0,0) signifie qu'il n'y a pas de prédécesseur 
	push!(heap, (0,current) )
	
	nextPos::Tuple{Int64, Int64} = (0,0)
	
	nbrSeen::Int64 = 0
	distFinal::Int64 = 0
	
	while !is_finished && !isempty(heap)
		(d,current) = pop!(heap)
		#on recupere le minimum 	
		
		
		
		if (!isSeen[current.pos[1], current.pos[2]])
		#sinon on l'a déja traité et via un chemin plus rapide
			#println( current.pos[1], " ", current.pos[2])
			isSeen[current.pos[1], current.pos[2]] = true
			nbrSeen += 1
			
			predessesorMap[CartesianIndex(current.pos)] = current.pred
			
			#on traite les voisins de current
			#en verifiant qu'on ne sorte pas de la carte
			#on fait cela pour les 4 voisins
			
			nextPos = (current.pos[1] -1, current.pos[2])
			if (current.pos[1] != 1 && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos) )
				
				push!(heap, (d + 1, DistPos(nextPos, current.pos ) ) )
			end
			
			nextPos = (current.pos[1] +1, current.pos[2])
			if (current.pos[1] != sizeX && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
			
				push!(heap, (d + 1, DistPos(nextPos, current.pos ) ) )
			end
			
			nextPos = (current.pos[1], current.pos[2] -1)
			if (current.pos[2] != 1 && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
			
				push!(heap, (d + 1, DistPos(nextPos, current.pos ) ) )
			end
			
			nextPos = (current.pos[1], current.pos[2] +1)
			if (current.pos[2] != sizeY && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
			
				push!(heap, (d + 1, DistPos(nextPos, current.pos ) ) )
			end
			
			if (current.pos === finish)
				is_finished = true;
				distFinal = d
			end
		end
	end
	
	return (distFinal, buildPath(predessesorMap, finish), nbrSeen)
end

function costDepl( map::Vector{String}, nextPos::Tuple{Int64, Int64})
	
	c::Char = map[nextPos[1]][nextPos[2]]
	
	if c == 'S'
		return 5
	elseif c == 'W'
		return 8
	else
		return 1
	end
end

function dijkstra(map::Vector{String}, start::Tuple{Int64, Int64}, finish::Tuple{Int64, Int64})
#precondition : la carte ne doit pas être vide
#reprise du code de floodfill avec quelques differences : la pondération des chemins n'est plus constante

	sizeX::Int64 = length(map)
	sizeY::Int64 = length(map[1])

	isSeen::Array{Bool} = fill(false, (sizeX, sizeY) )
	
	predessesorMap::Array{Tuple{Int64, Int64}} = Array{Tuple{Int64, Int64}}(undef, (sizeX, sizeY) )
	# permet de reconstruire le passage -> tableau des distances actuelles et des prédécesseurs
	
	heap::MutableBinaryHeap{Tuple{Int64, DistPos}} = MutableBinaryHeap{Tuple{Int64, DistPos}}( Base.By(first) )
	#tas Min/max qui permet de "trier" les valeurs
	
	is_finished::Bool = false
	
	current::DistPos = DistPos(start, (0,0) )
	# la position (0,0) signifie qu'il n'y a pas de prédécesseur 
	push!(heap, (0,current) )
	
	nextPos::Tuple{Int64, Int64} = (0,0)
	
	nbrSeen::Int64 = 0
	distFinal::Int64 = 0
	
	while !is_finished && !isempty(heap)
		(d,current) = pop!(heap)
		#on recupere le minimum 	
		
		
		
		if (!isSeen[current.pos[1], current.pos[2]])
		#sinon on l'a déja traité et via un chemin plus rapide
			#println( current.pos[1], " ", current.pos[2])
			isSeen[current.pos[1], current.pos[2]] = true
			nbrSeen += 1
			
			predessesorMap[CartesianIndex(current.pos)] = current.pred
			
			#on traite les voisins de current
			#en verifiant qu'on ne sorte pas de la carte
			#on fait cela pour les 4 voisins
			
			nextPos = (current.pos[1] -1, current.pos[2])
			if (current.pos[1] != 1 && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
				
				push!(heap, (d + costDepl(map, nextPos), DistPos(nextPos, current.pos ) ) )
			end
			
			nextPos = (current.pos[1] +1, current.pos[2])
			if (current.pos[1] != sizeX && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
			
				push!(heap, (d + costDepl(map, nextPos), DistPos(nextPos, current.pos ) ) )
			end
			
			nextPos = (current.pos[1], current.pos[2] -1)
			if (current.pos[2] != 1 && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
			
				push!(heap, (d + costDepl(map, nextPos), DistPos(nextPos, current.pos ) ) )
			end
			
			
			nextPos = (current.pos[1], current.pos[2] +1)
			if (current.pos[2] != sizeY && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
			
				push!(heap, (d + costDepl(map, nextPos), DistPos(nextPos, current.pos ) ) )
			end
			
			if (current.pos === finish)
				is_finished = true;
				distFinal = d
			end
		end
	end
	
	return (distFinal, buildPath(predessesorMap, finish), nbrSeen)
end

function manhattanDist( p1::Tuple{Int64, Int64}, p2::Tuple{Int64, Int64})
	
	return abs(p1[1] - p2[1]) + abs(p1[2] - p2[2] )
end

function Astar(map::Vector{String}, start::Tuple{Int64, Int64}, finish::Tuple{Int64, Int64})
#precondition : la carte ne doit pas être vide
#reprise du code de dijsktra avec quelques differences : on ajoute au cout selon lesquel les "voisins" sont triés la distance de manhattan
# 	comme celle ci était mémorisée via dans la pile (l'indice de tri), un ajout a du être fait dans 

	sizeX::Int64 = length(map)
	sizeY::Int64 = length(map[1])

	isSeen::Array{Bool} = fill(false, (sizeX, sizeY) )
	
	predessesorMap::Array{Tuple{Int64, Int64}} = Array{Tuple{Int64, Int64}}(undef, (sizeX, sizeY) )
	# permet de reconstruire le passage -> tableau des distances actuelles et des prédécesseurs
	
	heap::MutableBinaryHeap{Tuple{Float64, DistPos, Int64}} = MutableBinaryHeap{Tuple{Float64, DistPos, Int64}}( Base.By(first) )
	#tas Min/max qui permet de "trier" les valeurs
	#ajout d'une 3e valeur pour stoquer la distance calculée : la première on y ajoute la "distance de manhattan" ; inutilisable pour un calcul de distances
	
	is_finished::Bool = false
	
	current::DistPos = DistPos(start, (0,0) )
	# la position (0,0) signifie qu'il n'y a pas de prédécesseur 
	push!(heap, (0.0 ,current, 0) )
	
	nextPos::Tuple{Int64, Int64} = (0,0)
	
	nbrSeen::Int64 = 0
	distFinal = 0
	
	while !is_finished && !isempty(heap)
		(cost, current, d) = pop!(heap)
		#on recupere le minimum 	
		
		if (!isSeen[current.pos[1], current.pos[2]])
		#sinon on l'a déja traité et via un chemin plus rapide
			#println( current.pos[1], " ", current.pos[2])
			isSeen[current.pos[1], current.pos[2]] = true
			nbrSeen += 1
			
			predessesorMap[CartesianIndex(current.pos)] = current.pred
			
			#on traite les voisins de current
			#en verifiant qu'on ne sorte pas de la carte
			#on fait cela pour les 4 voisins
			
			nextPos = (current.pos[1] -1, current.pos[2])
			if (current.pos[1] != 1 && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
				
				push!(heap, (d + costDepl(map, nextPos) + manhattanDist(nextPos, finish), DistPos(nextPos, current.pos ), d + costDepl(map, nextPos) ) )
			end
			
			nextPos = (current.pos[1] +1, current.pos[2])
			if (current.pos[1] != sizeX && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
			
				push!(heap, (d + costDepl(map, nextPos) + manhattanDist(nextPos, finish), DistPos(nextPos, current.pos ), d + costDepl(map, nextPos) ) )
			end
			
			nextPos = (current.pos[1], current.pos[2] -1)
			if (current.pos[2] != 1 && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
			
				push!(heap, (d + costDepl(map, nextPos) + manhattanDist(nextPos, finish), DistPos(nextPos, current.pos ), d + costDepl(map, nextPos) ) )
			end
			
			nextPos = (current.pos[1], current.pos[2] +1)
			if (current.pos[2] != sizeY && !isSeen[CartesianIndex(nextPos)] && !estMur(map, nextPos))
			
				push!(heap, (d + costDepl(map, nextPos) + manhattanDist(nextPos, finish), DistPos(nextPos, current.pos ), d + costDepl(map, nextPos) ) )
			end
			
			if (current.pos === finish)
				is_finished = true;
				distFinal = d
			end
		end
	end
	
	return (distFinal, buildPath(predessesorMap, finish), nbrSeen)
end

function algoFloodFill(fname::String, d::Tuple{Int64, Int64}, f::Tuple{Int64, Int64} )
	map = parse("theglaive.map")
	floodfill(map, d, f)
end

function algoDijsktra(fname::String, d::Tuple{Int64, Int64}, f::Tuple{Int64, Int64} )
	map = parse("theglaive.map")
	dijkstra(map, d, f)
end

function algoAstar(fname::String, d::Tuple{Int64, Int64}, f::Tuple{Int64, Int64} )
	map = parse("theglaive.map")
	Astar(map, d, f)
end

function testAlgos(fname::String, d::Tuple{Int64, Int64}, f::Tuple{Int64, Int64})
	t1::Float64 = time()
	map::Vector{String} = parse("theglaive.map")
	t2::Float64 = time() - t1
	
	@printf "durée du chargement de la carte : %.5f \n\n" t2
	
	@printf "Floodfill : \n"
	
	t1 = time()
	(distFF, pathFF, seenFF) = floodfill(map, d, f)
	t2 = time() - t1
	
	@printf "Distance : %d ; cases visitées :  %d\n" distFF seenFF
	@printf "duree : %.5f \n\n" t2
	
	@printf "Dijsktra :\n"
	
	t1 = time()
	(distD, pathD, seenD) = dijkstra(map, d, f)
	t2 = time() - t1
	
	@printf "Distance : %d ; cases visitées :  %d\n" distD seenD
	@printf "duree : %5f \n\n" t2
	
	@printf "Astar :\n"
	
	t1 = time()
	(distA, pathA, seenA) = Astar(map, d, f)
	t2 = time() - t1
	
	@printf "Distance : %d ; cases visitées :  %d\n" distA seenA
	@printf "duree : %.5f \n\n" t2
end

function testExCours()
	testAlgos("theglaive.map", (189, 193), (226, 437))
end

function main()
	testExCours()
end
